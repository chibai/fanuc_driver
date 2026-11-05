#include <gtest/gtest.h>

#include <chrono>
#include <exception>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include <boost/asio.hpp>
#include "fanuc_driver/fanuc_driver.hpp"

using namespace std::chrono_literals;

namespace
{
struct Exchange
{
  std::string command;
  std::string response;
  std::chrono::milliseconds delay{ 0 };
  std::size_t fragment_size = 0;
  bool close = false;
  bool terminate_frame = true;
};

class Controller
{
public:
  explicit Controller(std::vector<Exchange> exchanges, std::string handshake_trailer = {})
    : listener_(context_, { boost::asio::ip::make_address("127.0.0.1"), 0 })
    , session_(context_, { boost::asio::ip::make_address("127.0.0.1"), 0 })
    , worker_([this, exchanges = std::move(exchanges),
               handshake_trailer = std::move(handshake_trailer)](std::stop_token stop) {
      try
      {
        auto handshake = accept(listener_, stop);
        const auto request = readLine(handshake);
        if (request != R"({"Communication":"FRC_Connect"})")
          throw std::runtime_error("Expected standard RMI handshake, got: " + request);
        writeBytes(handshake, R"({"Communication":"FRC_Connect","ErrorID":0,"PortNumber":)" +
                                  std::to_string(session_.local_endpoint().port()) +
                                  R"(,"MajorVersion":3,"MinorVersion":1})" + "\r\n" + handshake_trailer);
        handshake.close();
        auto connection = accept(session_, stop);
        for (const auto& exchange : exchanges)
        {
          auto request = readLine(connection);
          requests_.push_back(request);
          if (request.find(exchange.command) == std::string::npos)
            throw std::runtime_error("Unexpected RMI request: " + request);
          std::this_thread::sleep_for(exchange.delay);
          if (!exchange.response.empty())
          {
            const auto bytes = exchange.response + (exchange.terminate_frame ? "\r\n" : "");
            if (exchange.fragment_size == 0)
              writeBytes(connection, bytes);
            else
              for (std::size_t offset = 0; offset < bytes.size(); offset += exchange.fragment_size)
              {
                writeBytes(connection, bytes.substr(offset, exchange.fragment_size));
                std::this_thread::sleep_for(1ms);
              }
          }
          if (exchange.close)
            return;
        }
        while (!stop.stop_requested())
          std::this_thread::sleep_for(1ms);
      }
      catch (...)
      {
        error_ = std::current_exception();
      }
    })
  {
  }

  uint16_t port() const
  {
    return listener_.local_endpoint().port();
  }

  void finish()
  {
    worker_.request_stop();
    worker_.join();
    if (error_)
      std::rethrow_exception(error_);
  }

  const std::vector<std::string>& requests() const
  {
    return requests_;
  }

private:
  static boost::asio::ip::tcp::socket accept(boost::asio::ip::tcp::acceptor& listener, std::stop_token stop)
  {
    listener.non_blocking(true);
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (!stop.stop_requested() && std::chrono::steady_clock::now() < deadline)
    {
      boost::system::error_code error;
      auto connection = listener.accept(error);
      if (!error)
      {
        connection.non_blocking(true);
        return connection;
      }
      if (error != boost::asio::error::would_block && error != boost::asio::error::try_again)
        throw boost::system::system_error(error);
      std::this_thread::sleep_for(1ms);
    }
    throw std::runtime_error("Timed out accepting mock RMI connection");
  }

  static std::string readLine(boost::asio::ip::tcp::socket& connection)
  {
    std::string line;
    char ch;
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (line.size() < 16384)
    {
      boost::system::error_code error;
      const auto size = connection.read_some(boost::asio::buffer(&ch, 1), error);
      if (error == boost::asio::error::would_block || error == boost::asio::error::try_again)
      {
        if (std::chrono::steady_clock::now() >= deadline)
          throw std::runtime_error("Timed out reading mock RMI request");
        std::this_thread::sleep_for(1ms);
        continue;
      }
      if (error || size != 1)
        throw std::runtime_error("Unable to read mock RMI request: " + error.message());
      line += ch;
      if (line.ends_with("\r\n"))
      {
        line.resize(line.size() - 2);
        return line;
      }
    }
    throw std::runtime_error("Mock RMI request exceeds limit");
  }

  static void writeBytes(boost::asio::ip::tcp::socket& connection, const std::string& bytes)
  {
    connection.non_blocking(false);
    boost::asio::write(connection, boost::asio::buffer(bytes));
    connection.non_blocking(true);
  }

  boost::asio::io_context context_;
  boost::asio::ip::tcp::acceptor listener_;
  boost::asio::ip::tcp::acceptor session_;
  std::vector<std::string> requests_;
  std::exception_ptr error_;
  std::jthread worker_;
};

static_assert(!std::is_copy_constructible_v<Fanuc_driver>);
static_assert(!std::is_move_constructible_v<Fanuc_driver>);
}  // namespace

TEST(FanucDriver, StandardRmiSessionAndControl)
{
  Controller controller({
      { "FRC_Initialize", R"({"Command":"FRC_Initialize","ErrorID":0})" },
      { "FRC_GetStatus",
        R"({"Command":"FRC_GetStatus","ErrorID":0,"ServoReady":1,"TPMode":0,"RMIMotionStatus":1,"ProgramStatus":0,"SingleStepMode":0,"NumberUTool":10,"NextSequenceID":1,"NumberUFrame":9,"Override":20})" },
      { "FRC_SetOverRide", R"({"Command":"FRC_SetOverRide","ErrorID":0})" },
      { "FRC_Pause", R"({"Command":"FRC_Pause","ErrorID":0})" },
      { "FRC_Continue", R"({"Command":"FRC_Continue","ErrorID":0})" },
      { "FRC_Abort", R"({"Command":"FRC_Abort","ErrorID":0})" },
      { "FRC_Reset", R"({"Command":"FRC_Reset","ErrorID":0})" },
      { "FRC_Disconnect", R"({"Communication":"FRC_Disconnect","ErrorID":0})" },
  });
  Fanuc_driver driver("127.0.0.1", controller.port());
  EXPECT_EQ(driver.connect().MajorVersion, 3);
  driver.initializeRemoteMotion(1.0, 1, "ON", "OFF");
  EXPECT_EQ(driver.getStatus().ServoReady, 1);
  driver.setSpeedOverride(20);
  driver.pause();
  driver.resume();
  driver.abort();
  driver.reset();
  driver.disconnect();
  ASSERT_NO_THROW(controller.finish());
  EXPECT_NE(controller.requests()[0].find(R"("GroupMask":1)"), std::string::npos);
  EXPECT_NE(controller.requests()[2].find(R"("Value":20)"), std::string::npos);
}

TEST(FanucDriver, RegistersIoAndVariables)
{
  Controller controller({
      { "FRC_ReadRegister", R"({"Command":"FRC_ReadRegister","ErrorID":0,"DataType":"REAL","RegisterValue":1.5})" },
      { "FRC_WriteRegister", R"({"Command":"FRC_WriteRegister","ErrorID":0})" },
      { "FRC_ReadDIN", R"({"Command":"FRC_ReadDIN","ErrorID":0,"PortNumber":1,"PortValue":1})" },
      { "FRC_WriteDOUT", R"({"Command":"FRC_WriteDOUT","ErrorID":0})" },
      { "FRC_ReadIOPort", R"({"Command":"FRC_ReadIOPort","ErrorID":0,"PortType":"AI","PortNumber":2,"PortValue":2.5})" },
      { "FRC_WriteIOPort", R"({"Command":"FRC_WriteIOPort","ErrorID":0,"PortType":"AO","PortNumber":2})" },
      { "FRC_ReadVariable",
        R"({"Command":"FRC_ReadVariable","ErrorID":0,"VariableName":"$TEST","VariableType":"Integer","VariableValue":7})" },
      { "FRC_WriteVariable", R"({"Command":"FRC_WriteVariable","ErrorID":0,"VariableName":"$TEST"})" },
  });
  Fanuc_driver driver("127.0.0.1", controller.port());
  driver.connect();
  EXPECT_FLOAT_EQ(std::get<float>(driver.readNumericRegister(1).RegisterValue), 1.5f);
  driver.writeNumericRegister(1, 2.5f);
  EXPECT_EQ(driver.readDigitalInputPort(1).PortValue, 1);
  driver.writeDigitalOutputPort(1, true);
  EXPECT_FLOAT_EQ(std::get<float>(driver.readIOPort("AI", 2).PortValue), 2.5f);
  driver.writeIOPort(2, "AO", 3.5f);
  EXPECT_EQ(std::get<int>(driver.readVariablePacket("$TEST").VariableValue), 7);
  driver.writeVariablePacket("$TEST", 8);
  ASSERT_NO_THROW(controller.finish());
  EXPECT_NE(controller.requests()[1].find(R"("RegisterValue":2.5)"), std::string::npos);
  EXPECT_NE(controller.requests()[7].find(R"("VariableValue":8)"), std::string::npos);
}

TEST(FanucDriver, MotionRepliesAndSequenceTracking)
{
  Controller controller({
      { "FRC_Initialize", R"({"Command":"FRC_Initialize","ErrorID":0})" },
      { "FRC_JointMotionJRep", R"({"Instruction":"FRC_JointMotionJRep","ErrorID":0,"SequenceID":1})" },
      { "FRC_LinearRelative", R"({"Instruction":"FRC_LinearRelative","ErrorID":0,"SequenceID":2})" },
      { "FRC_WaitTime", R"({"Instruction":"FRC_WaitTime","ErrorID":0,"SequenceID":3})" },
      { "FRC_Call", R"({"Instruction":"FRC_Call","ErrorID":0,"SequenceID":4})" },
  });
  Fanuc_driver driver("127.0.0.1", controller.port());
  driver.connect();
  driver.initializeRemoteMotion();
  EXPECT_EQ(driver.getRemainingBufferSize(), 8);
  rmi::JointMotionJRepPacket::Request joint{};
  joint.SpeedType = "Percent";
  joint.Speed = 10;
  joint.TermType = "FINE";
  EXPECT_EQ(driver.sendJointMotion(joint).SequenceID, 1);
  rmi::LinearRelativePacket::Request linear{};
  linear.Position.Z = 10;
  linear.SpeedType = "mmSec";
  linear.Speed = 10;
  linear.TermType = "FINE";
  EXPECT_EQ(driver.sendRMIPacket<rmi::LinearRelativePacket>(linear).SequenceID, 2);
  rmi::WaitForTimePacket::Request wait{};
  driver.sendRMIPacketNonBlocking(wait);
  EXPECT_EQ(wait.SequenceID, 3);
  EXPECT_EQ(driver.programCall("TEST", 1.0, { { "Integer", 7 } }).SequenceID, 4);
  EXPECT_EQ(driver.getLastInstructionResponse()->SequenceID, 4);
  EXPECT_EQ(driver.getRemainingBufferSize(), 8);
  ASSERT_NO_THROW(controller.finish());
  EXPECT_NE(controller.requests()[4].find(R"("ParamValue1":7)"), std::string::npos);
}

TEST(FanucDriver, ControllerErrorsAndTimeoutsAreReported)
{
  Controller controller({
      { "FRC_Reset", R"({"Command":"FRC_Reset","ErrorID":2556940})" },
      { "FRC_GetStatus", "" },
  });
  Fanuc_driver driver("127.0.0.1", controller.port());
  driver.connect();
  try
  {
    driver.reset();
    FAIL() << "Controller error must throw";
  }
  catch (const std::runtime_error& error)
  {
    EXPECT_NE(std::string(error.what()).find("Cannot Reset Controller"), std::string::npos);
  }
  EXPECT_THROW(driver.getStatus(0.01), std::runtime_error);
  ASSERT_NO_THROW(controller.finish());
}

TEST(FanucDriver, FragmentedAndLargeResponses)
{
  const std::string alarm(8192, 'a');
  Controller controller({
      { "FRC_Reset", R"({"Command":"FRC_Reset","ErrorID":0})", 0ms, 1 },
      { "FRC_ReadError", R"({"Command":"FRC_ReadError","ErrorID":0,"ErrorData":")" + alarm + R"("})" },
  });
  Fanuc_driver driver("localhost", controller.port());
  driver.connect();
  EXPECT_EQ(driver.reset().ErrorID, 0);
  EXPECT_EQ(driver.readError().ErrorData, alarm);
  ASSERT_NO_THROW(controller.finish());
}

TEST(FanucDriver, NewSessionDiscardsHandshakeFragment)
{
  Controller controller({ { "FRC_Reset", R"({"Command":"FRC_Reset","ErrorID":0})" } }, R"({"stale":)");
  Fanuc_driver driver("127.0.0.1", controller.port());
  EXPECT_EQ(driver.connect().MajorVersion, 3);
  EXPECT_EQ(driver.reset().ErrorID, 0);
  ASSERT_NO_THROW(controller.finish());
}

TEST(FanucDriver, ResponseTimeoutPreservesPendingRead)
{
  Controller controller({
      { "FRC_Reset", R"({"Command":"FRC_Reset","ErrorID":0})", 100ms },
      { "FRC_Pause", R"({"Command":"FRC_Pause","ErrorID":0})" },
  });
  Fanuc_driver driver("127.0.0.1", controller.port());
  driver.connect();
  const auto start = std::chrono::steady_clock::now();
  EXPECT_THROW(driver.reset(0.01), std::runtime_error);
  EXPECT_LT(std::chrono::steady_clock::now() - start, 80ms);
  EXPECT_EQ(driver.pause().ErrorID, 0);
  ASSERT_NO_THROW(controller.finish());
}

TEST(FanucDriver, UnlimitedAndLargeTimeoutsWaitForData)
{
  Controller controller({
      { "FRC_Reset", R"({"Command":"FRC_Reset","ErrorID":0})", 20ms },
      { "FRC_Pause", R"({"Command":"FRC_Pause","ErrorID":0})", 20ms },
  });
  Fanuc_driver driver("127.0.0.1", controller.port());
  driver.connect();
  EXPECT_EQ(driver.reset(-1.0).ErrorID, 0);
  EXPECT_EQ(driver.pause(std::numeric_limits<double>::max()).ErrorID, 0);
  ASSERT_NO_THROW(controller.finish());
}

TEST(FanucDriver, NonFiniteTimeoutsAreReported)
{
  Controller controller({
      { "FRC_Reset", "" },
      { "FRC_Reset", "" },
      { "FRC_Reset", "" },
      { "FRC_Pause", R"({"Command":"FRC_Pause","ErrorID":0})" },
  });
  Fanuc_driver driver("127.0.0.1", controller.port());
  driver.connect();
  EXPECT_THROW(driver.reset(std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
  EXPECT_THROW(driver.reset(std::numeric_limits<double>::infinity()), std::invalid_argument);
  EXPECT_THROW(driver.reset(-std::numeric_limits<double>::infinity()), std::invalid_argument);
  EXPECT_EQ(driver.pause().ErrorID, 0);
  ASSERT_NO_THROW(controller.finish());
}

TEST(FanucDriver, PushChecksDoNotWaitForData)
{
  Controller controller({ { "FRC_Reset", R"({"Command":"FRC_Reset","ErrorID":0})" } });
  Fanuc_driver driver("127.0.0.1", controller.port());
  driver.connect();
  driver.reset();
  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(driver.checkSystemFault());
  EXPECT_FALSE(driver.checkTimeoutTerminate());
  EXPECT_FALSE(driver.checkCommunicationPacket());
  EXPECT_FALSE(driver.checkUnknownPacket());
  EXPECT_FALSE(driver.getLastInstructionResponse());
  EXPECT_LT(std::chrono::steady_clock::now() - start, 100ms);
  ASSERT_NO_THROW(controller.finish());
}

TEST(FanucDriver, CompleteResponseBeforeEofIsDelivered)
{
  Controller controller({ { "FRC_Reset", R"({"Command":"FRC_Reset","ErrorID":0})", 0ms, 0, true } });
  Fanuc_driver driver("127.0.0.1", controller.port());
  driver.connect();
  EXPECT_EQ(driver.reset().ErrorID, 0);
  ASSERT_NO_THROW(controller.finish());
  EXPECT_THROW(driver.checkSystemFault(), std::runtime_error);
}

TEST(FanucDriver, EofWithoutCompleteResponseIsReported)
{
  Controller controller({ { "FRC_Reset", R"({"Command":)", 0ms, 0, true, false } });
  Fanuc_driver driver("127.0.0.1", controller.port());
  driver.connect();
  try
  {
    driver.reset();
    FAIL() << "An incomplete response followed by EOF must throw";
  }
  catch (const std::runtime_error& error)
  {
    EXPECT_NE(std::string(error.what()).find("closed by the controller"), std::string::npos);
  }
  ASSERT_NO_THROW(controller.finish());
}

TEST(FanucDriver, RefusedConnectionIsReported)
{
  boost::asio::io_context context;
  boost::asio::ip::tcp::acceptor listener(context, { boost::asio::ip::make_address("127.0.0.1"), 0 });
  const auto port = listener.local_endpoint().port();
  listener.close();
  EXPECT_THROW(Fanuc_driver("127.0.0.1", port), std::runtime_error);
}

TEST(FanucDriver, ProgramCallMapsTenArgumentsAndRejectsOverflow)
{
  Controller controller({
      { "FRC_Call", R"({"Instruction":"FRC_Call","ErrorID":0,"SequenceID":1})" },
      { "FRC_Call", R"({"Instruction":"FRC_Call","ErrorID":0,"SequenceID":2})" },
  });
  Fanuc_driver driver("127.0.0.1", controller.port());
  driver.connect();
  std::vector<rmi::RMICallParam> params;
  for (int i = 1; i <= 10; ++i)
    params.emplace_back("Integer", i);
  EXPECT_EQ(driver.programCall("TEST", 1.0, params).SequenceID, 1);
  params.emplace_back("Integer", 11);
  EXPECT_THROW(driver.programCallNonBlocking("TEST", params), std::invalid_argument);
  EXPECT_EQ(driver.programCall("NO_ARGS").SequenceID, 2);
  ASSERT_NO_THROW(controller.finish());
  for (int i = 1; i <= 10; ++i)
    EXPECT_NE(controller.requests()[0].find("\"ParamValue" + std::to_string(i) + "\":" + std::to_string(i)),
              std::string::npos);
  EXPECT_EQ(controller.requests()[1].find("ParamType"), std::string::npos);
}

TEST(FanucDriver, CoordinatesPositionsAndPayloads)
{
  const std::string frame = R"({"X":10,"Y":0,"Z":0,"W":0,"P":0,"R":0})";
  const std::string joints = R"({"J1":15,"J2":0,"J3":0,"J4":0,"J5":0,"J6":0,"J7":0,"J8":0,"J9":0})";
  const std::string configuration =
      R"({"UToolNumber":1,"UFrameNumber":2,"Front":1,"Up":0,"Left":0,"Flip":0,"Turn4":0,"Turn5":0,"Turn6":0})";
  const std::string position = R"({"X":20,"Y":0,"Z":0,"W":0,"P":0,"R":0,"Ext1":0,"Ext2":0,"Ext3":0})";
  Controller controller({
      { "FRC_GetUFrameUTool", R"({"Command":"FRC_GetUFrameUTool","ErrorID":0,"UFrameNumber":2,"UToolNumber":1})" },
      { "FRC_SetUFrameUTool", R"({"Command":"FRC_SetUFrameUTool","ErrorID":0})" },
      { "FRC_ReadUFrameData", R"({"Command":"FRC_ReadUFrameData","ErrorID":0,"FrameNumber":2,"Frame":)" + frame + "}" },
      { "FRC_WriteUFrameData", R"({"Command":"FRC_WriteUFrameData","ErrorID":0})" },
      { "FRC_ReadUToolData", R"({"Command":"FRC_ReadUToolData","ErrorID":0,"ToolNumber":1,"Frame":)" + frame + "}" },
      { "FRC_WriteUToolData", R"({"Command":"FRC_WriteUToolData","ErrorID":0})" },
      { "FRC_ReadJointAngles",
        R"({"Command":"FRC_ReadJointAngles","ErrorID":0,"TimeTag":1,"JointAngle":)" + joints + "}" },
      { "FRC_ReadCartesianPosition",
        R"({"Command":"FRC_ReadCartesianPosition","ErrorID":0,"TimeTag":1,"Configuration":)" + configuration +
            R"(,"Position":)" + position + "}" },
      { "FRC_ReadTCPSpeed", R"({"Command":"FRC_ReadTCPSpeed","ErrorID":0,"TimeTag":1,"Speed":10})" },
      { "FRC_ReadPositionRegister",
        R"({"Command":"FRC_ReadPositionRegister","ErrorID":0,"RegisterNumber":1,"Representation":"Joint","JointAngle":)" +
            joints + "}" },
      { "FRC_WritePositionRegister", R"({"Command":"FRC_WritePositionRegister","ErrorID":0,"RegisterNumber":1})" },
      { "FRC_GetPayloadID", R"({"Command":"FRC_GetPayloadID","ErrorID":0,"ScheduleNumber":1})" },
      { "FRC_SetPayloadID", R"({"Command":"FRC_SetPayloadID","ErrorID":0})" },
      { "FRC_GetPayloadValue",
        R"({"Command":"FRC_GetPayloadValue","ErrorID":0,"Mass":2,"CG_X":0,"CG_Y":0,"CG_Z":0,"IN_X":0,"IN_Y":0,"IN_Z":0})" },
      { "FRC_SetPayloadValue", R"({"Command":"FRC_SetPayloadValue","ErrorID":0})" },
      { "FRC_GetPayloadComp",
        R"({"Command":"FRC_GetPayloadComp","ErrorID":0,"Mass":3,"CG_X":0,"CG_Y":0,"CG_Z":0,"IN_X":0,"IN_Y":0,"IN_Z":0})" },
      { "FRC_SetPayloadComp", R"({"Command":"FRC_SetPayloadComp","ErrorID":0})" },
  });
  Fanuc_driver driver("127.0.0.1", controller.port());
  driver.connect();
  EXPECT_EQ(driver.getUFrameUTool().UFrameNumber, 2);
  driver.setUFrameUTool(2, 1, 1.0, 1);
  EXPECT_FLOAT_EQ(driver.readUFrameData(2).Frame.X, 10);
  rmi::FrameData data{};
  data.X = 11;
  driver.writeUFrameData(2, data);
  EXPECT_FLOAT_EQ(driver.readUToolData(1).Frame.X, 10);
  driver.writeUToolData(1, data);
  EXPECT_FLOAT_EQ(driver.readJointAngles(1).JointAngle.J1, 15);
  EXPECT_FLOAT_EQ(driver.getCartesianPosition().Position.X, 20);
  EXPECT_FLOAT_EQ(driver.getTCPSpeed().Speed, 10);
  EXPECT_FLOAT_EQ(driver.readPositionRegister(1).JointAngle->J1, 15);
  rmi::JointAngleData target{};
  target.J1 = 30;
  driver.writePositionRegister(1, "Joint", {}, {}, target);
  EXPECT_EQ(driver.getPayloadSchedule().ScheduleNumber, 1);
  driver.setPayloadSchedule(1);
  EXPECT_FLOAT_EQ(driver.getPayloadValue(1).Mass, 2);
  driver.setPayloadValue(1, 2, 0, 0, 0, true, 1, 2, 3);
  EXPECT_FLOAT_EQ(driver.getPayloadComp(1).Mass, 3);
  driver.setPayloadComp(1, 3, 0, 0, 0, 1, 2, 3);
  ASSERT_NO_THROW(controller.finish());
  EXPECT_NE(controller.requests()[1].find(R"("Group":1)"), std::string::npos);
  EXPECT_NE(controller.requests()[10].find(R"("J1":30)"), std::string::npos);
  EXPECT_NE(controller.requests()[14].find(R"("IN_Z":3)"), std::string::npos);
}

TEST(FanucDriver, DiagnosticsAndPushPackets)
{
  Controller controller({
      { "FRC_ReadError",
        R"({"Command":"FRC_ReadError","ErrorID":0,"Count":2,"ErrorData":"alarm","ErrorData2":"detail"})" },
      { "FRC_GetExtStatus",
        R"({"Command":"FRC_GetExtStatus","ErrorID":0,"InMotion":0,"DrivesPowered":1,"GenOverride":20})" },
      { "FRC_Call", R"({"Instruction":"FRC_Call","ErrorID":0,"SequenceID":1})"
                    "\r\n"
                    R"({"Communication":"FRC_SystemFault","SequenceID":1})"
                    "\r\n"
                    R"({"Communication":"FRC_Terminate"})"
                    "\r\n"
                    R"({"Communication":"FRC_AsbnReady"})"
                    "\r\n"
                    R"({"Command":"Unknown","ErrorID":2556941})" },
      { "FRC_ReadError", R"({"Command":"FRC_ReadError","ErrorID":0,"ErrorData":")"
                         "\x83\x65\x83\x58\x83\x67"
                         R"("})" },
  });
  Fanuc_driver driver("127.0.0.1", controller.port());
  driver.connect();
  EXPECT_EQ(driver.readError(1.0, 2).ErrorData2, "detail");
  EXPECT_EQ(driver.getExtendedStatus().DrivesPowered, 1);
  EXPECT_EQ(driver.programCallNonBlocking("TEST").SequenceID, 1);
  std::optional<rmi::SystemFaultPacket> fault;
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (!fault && std::chrono::steady_clock::now() < deadline)
    fault = driver.checkSystemFault();
  ASSERT_TRUE(fault);
  EXPECT_EQ(fault->SequenceID, 1);
  EXPECT_TRUE(driver.checkTimeoutTerminate());
  EXPECT_TRUE(driver.checkCommunicationPacket());
  EXPECT_TRUE(driver.checkUnknownPacket());
  EXPECT_FALSE(driver.checkSystemFault());
  EXPECT_EQ(driver.getLastInstructionResponse()->SequenceID, 1);
  EXPECT_EQ(driver.getErrorMessageString(2556940), "Cannot Reset Controller.");
  driver.setEncoding("SHIFT-JIS");
  EXPECT_EQ(driver.readError().ErrorData, "\xe3\x83\x86\xe3\x82\xb9\xe3\x83\x88");
  ASSERT_NO_THROW(controller.finish());
}

template <typename Packet>
class RmiInstruction : public testing::Test
{
};

using Instructions = testing::Types<rmi::JointMotionPacket, rmi::JointMotionJRepPacket, rmi::JointRelativePacket,
                                    rmi::JointRelativeJRepPacket, rmi::LinearMotionPacket, rmi::LinearMotionJRepPacket,
                                    rmi::LinearRelativePacket, rmi::LinearRelativeJRepPacket, rmi::CircularMotionPacket,
                                    rmi::CircularRelativePacket, rmi::SplineMotionPacket, rmi::SplineMotionJRepPacket,
                                    rmi::WaitForDINPacket, rmi::WaitForTimePacket, rmi::SetUFramePacket,
                                    rmi::SetToolFramePacket, rmi::SetPayloadInstructionPacket>;
TYPED_TEST_SUITE(RmiInstruction, Instructions);

TYPED_TEST(RmiInstruction, BlockingAndNonBlocking)
{
  typename TypeParam::Request request{};
  const auto response = [&request](int sequence) {
    return R"({"Instruction":")" + request.Instruction + R"(","ErrorID":0,"SequenceID":)" + std::to_string(sequence) +
           "}";
  };
  Controller controller({
      { "FRC_Initialize", R"({"Command":"FRC_Initialize","ErrorID":0})" },
      { request.Instruction, response(0) + "\r\n" + response(1) },
      { request.Instruction, response(2) },
  });
  Fanuc_driver driver("127.0.0.1", controller.port());
  driver.connect();
  driver.initializeRemoteMotion();
  EXPECT_EQ(driver.sendRMIPacket<TypeParam>(request).SequenceID, 1);
  driver.sendRMIPacketNonBlocking(request);
  EXPECT_EQ(request.SequenceID, 2);
  std::optional<rmi::InstructionResponse> last;
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  do
  {
    last = driver.getLastInstructionResponse();
  } while ((!last || last->SequenceID != 2) && std::chrono::steady_clock::now() < deadline);
  ASSERT_TRUE(last);
  EXPECT_EQ(last->SequenceID, 2);
  EXPECT_EQ(driver.getRemainingBufferSize(), 8);
  ASSERT_NO_THROW(controller.finish());
}
