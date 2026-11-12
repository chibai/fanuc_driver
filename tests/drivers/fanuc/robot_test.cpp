#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <exception>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include <boost/asio.hpp>
#include "drivers/fanuc/detail/rmi_driver.hpp"
#include "robot_driver/drivers/fanuc/robot.hpp"

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
  explicit Controller(std::vector<Exchange> exchanges, std::string handshake_trailer = {}, bool refuse_session = false)
    : listener_(context_, { boost::asio::ip::make_address("127.0.0.1"), 0 })
    , session_(context_, { boost::asio::ip::make_address("127.0.0.1"), 0 })
    , worker_([this, exchanges = std::move(exchanges), refuse_session,
               handshake_trailer = std::move(handshake_trailer)](std::stop_token stop) {
      try
      {
        auto handshake = accept(listener_, stop);
        const auto request = readLine(handshake);
        if (request != R"({"Communication":"FRC_Connect"})")
          throw std::runtime_error("Expected standard RMI handshake, got: " + request);
        const auto session_port = session_.local_endpoint().port();
        if (refuse_session)
          session_.close();
        writeBytes(handshake, R"({"Communication":"FRC_Connect","ErrorID":0,"PortNumber":)" +
                                  std::to_string(session_port) + R"(,"MajorVersion":3,"MinorVersion":1})" + "\r\n" +
                                  handshake_trailer);
        handshake.close();
        if (refuse_session)
          return;
        auto connection = accept(session_, stop);
        for (const auto& exchange : exchanges)
        {
          auto request = readLine(connection);
          {
            std::scoped_lock lock(request_mutex_);
            requests_.push_back(request);
          }
          request_changed_.notify_all();
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

  bool waitForRequests(std::size_t count)
  {
    std::unique_lock lock(request_mutex_);
    return request_changed_.wait_for(lock, 1s, [&]() { return requests_.size() >= count; });
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
  std::mutex request_mutex_;
  std::condition_variable request_changed_;
  std::exception_ptr error_;
  std::jthread worker_;
};

static_assert(!std::is_copy_constructible_v<Fanuc_driver>);
static_assert(!std::is_move_constructible_v<Fanuc_driver>);
static_assert(std::is_base_of_v<Robot_driver, Fanuc_driver>);
static_assert(!std::is_abstract_v<Fanuc_driver>);
static_assert(std::is_same_v<decltype(std::declval<Fanuc_driver&>().connect()), ResponseStatus>);
static_assert(std::is_same_v<decltype(std::declval<Fanuc_driver&>().getStatus()), Response<RobotStatus>>);
static_assert(std::is_same_v<decltype(std::declval<Fanuc_driver&>().readDigitalInputPort(1)),
                             std::expected<bool, ResponseStatus>>);
}  // namespace

TEST(FanucDriver, BrandIndependentInterface)
{
  Controller controller({
      { "FRC_Initialize", R"({"Command":"FRC_Initialize","ErrorID":0})" },
      { "FRC_GetStatus",
        R"({"Command":"FRC_GetStatus","ErrorID":0,"ServoReady":1,"TPMode":0,"RMIMotionStatus":1,"ProgramStatus":0,"SingleStepMode":0,"NumberUTool":10,"NextSequenceID":1,"NumberUFrame":9,"Override":20})" },
      { "FRC_SetOverRide", R"({"Command":"FRC_SetOverRide","ErrorID":0})" },
      { "FRC_ReadDIN", R"({"Command":"FRC_ReadDIN","ErrorID":0,"PortNumber":7,"PortValue":1})" },
      { "FRC_ReadDIN", R"({"Command":"FRC_ReadDIN","ErrorID":0,"PortNumber":8,"PortValue":0})" },
      { "FRC_WriteDOUT", R"({"Command":"FRC_WriteDOUT","ErrorID":0})" },
      { "FRC_WriteDOUT", R"({"Command":"FRC_WriteDOUT","ErrorID":0})" },
      { "FRC_Pause", R"({"Command":"FRC_Pause","ErrorID":0})" },
      { "FRC_Continue", R"({"Command":"FRC_Continue","ErrorID":0})" },
      { "FRC_Abort", R"({"Command":"FRC_Abort","ErrorID":0})" },
      { "FRC_Reset", R"({"Command":"FRC_Reset","ErrorID":0})" },
      { "FRC_Disconnect", R"({"Communication":"FRC_Disconnect","ErrorID":0})" },
  });
  std::unique_ptr<Robot_driver> driver = std::make_unique<Fanuc_driver>("127.0.0.1", controller.port());
  ASSERT_TRUE(driver->connect().ok());
  ASSERT_TRUE(driver->initializeRemoteMotion().ok());
  const auto status = driver->getStatus();
  ASSERT_TRUE(status.ok());
  EXPECT_TRUE(status.data->servo_ready);
  EXPECT_EQ(status.data->speed_override, 20);
  EXPECT_TRUE(driver->setSpeedOverride(35).ok());
  const auto on = driver->readDigitalInputPort(7);
  const auto off = driver->readDigitalInputPort(8);
  ASSERT_TRUE(on.has_value());
  ASSERT_TRUE(off.has_value());
  EXPECT_TRUE(*on);
  EXPECT_FALSE(*off);
  EXPECT_TRUE(driver->writeDigitalOutputPort(9, true).ok());
  EXPECT_TRUE(driver->writeDigitalOutputPort(10, false).ok());
  EXPECT_TRUE(driver->pause().ok());
  EXPECT_TRUE(driver->resume().ok());
  EXPECT_TRUE(driver->abort().ok());
  EXPECT_TRUE(driver->reset().ok());
  EXPECT_TRUE(driver->disconnect().ok());
  driver.reset();
  ASSERT_NO_THROW(controller.finish());
  EXPECT_NE(controller.requests()[2].find(R"("Value":35)"), std::string::npos);
  EXPECT_NE(controller.requests()[3].find(R"("PortNumber":7)"), std::string::npos);
  EXPECT_NE(controller.requests()[4].find(R"("PortNumber":8)"), std::string::npos);
  EXPECT_NE(controller.requests()[5].find(R"("PortNumber":9)"), std::string::npos);
  EXPECT_NE(controller.requests()[5].find(R"("PortValue":"ON")"), std::string::npos);
  EXPECT_NE(controller.requests()[6].find(R"("PortNumber":10)"), std::string::npos);
  EXPECT_NE(controller.requests()[6].find(R"("PortValue":"OFF")"), std::string::npos);
}

TEST(FanucDriver, BrandIndependentInterfaceReturnsErrors)
{
  Controller controller({
      { "FRC_Pause", R"({"Command":"FRC_Pause","ErrorID":2556938})" },
      { "FRC_GetStatus", "", 0ms, 0, true },
  });
  Fanuc_driver fanuc("127.0.0.1", controller.port());
  Robot_driver& driver = fanuc;
  ASSERT_TRUE(driver.connect().ok());
  const auto pause = driver.pause();
  EXPECT_EQ(pause.code(), 9999);
  EXPECT_NE(pause.detail.find("TP Program is Not Paused"), std::string::npos);
  const auto status = driver.getStatus();
  EXPECT_EQ(status.status.value, ResponseCode::CONNECTION_ERROR);
  EXPECT_FALSE(status.data);
  ASSERT_NO_THROW(controller.finish());
}

TEST(FanucDriver, ConcreteInterfaceReturnsOnlyCoreTypes)
{
  Controller controller({
      { "FRC_Reset", R"({"Command":"FRC_Reset","ErrorID":2556940})" },
      { "FRC_Pause", R"({"Command":"FRC_Pause","ErrorID":42})" },
      { "FRC_ReadDIN", R"({"Command":"FRC_ReadDIN","ErrorID":0,"PortNumber":1,"PortValue":0})" },
      { "FRC_ReadDIN", R"({"Command":"FRC_ReadDIN","ErrorID":2556966})" },
      { "FRC_GetStatus", R"({"Command":"FRC_GetStatus","ErrorID":2556942})" },
      { "FRC_GetStatus", "" },
  });
  Fanuc_driver driver("127.0.0.1", controller.port());
  ASSERT_TRUE(driver.connect().ok());

  const auto reset = driver.reset();
  EXPECT_FALSE(reset.ok());
  EXPECT_EQ(reset.code(), 9999);
  EXPECT_NE(reset.detail.find("Cannot Reset Controller"), std::string::npos);
  const auto pause = driver.pause();
  EXPECT_EQ(pause.code(), 9999);
  EXPECT_NE(pause.detail.find("Unknown Error Code: 42"), std::string::npos);

  const auto off = driver.readDigitalInputPort(1);
  ASSERT_TRUE(off.has_value());
  EXPECT_FALSE(*off);
  const auto input_error = driver.readDigitalInputPort(1);
  ASSERT_FALSE(input_error.has_value());
  EXPECT_EQ(input_error.error().code(), 9999);
  EXPECT_NE(input_error.error().detail.find("Invalid Port Number"), std::string::npos);
  const auto status_error = driver.getStatus();
  EXPECT_EQ(status_error.status.code(), 9999);
  EXPECT_FALSE(status_error.data);
  EXPECT_NE(status_error.status.detail.find("RMI Command Fail"), std::string::npos);
  const auto timeout = driver.getStatus(0.01);
  EXPECT_EQ(timeout.status.value, ResponseCode::WAITING_TIMEOUT);
  EXPECT_FALSE(timeout.data);
  ASSERT_NO_THROW(controller.finish());
}

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
  FanucRmiDriver driver("127.0.0.1", controller.port());
  EXPECT_EQ(driver.connect().data.value().MajorVersion, 3);
  ASSERT_TRUE(driver.initializeRemoteMotion(1.0, 1, "ON", "OFF").ok());
  EXPECT_EQ(driver.getStatus().data.value().ServoReady, 1);
  ASSERT_TRUE(driver.setSpeedOverride(20).ok());
  ASSERT_TRUE(driver.pause().ok());
  ASSERT_TRUE(driver.resume().ok());
  ASSERT_TRUE(driver.abort().ok());
  ASSERT_TRUE(driver.reset().ok());
  ASSERT_TRUE(driver.disconnect().ok());
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
  FanucRmiDriver driver("127.0.0.1", controller.port());
  ASSERT_TRUE(driver.connect().ok());
  EXPECT_FLOAT_EQ(std::get<float>(driver.readNumericRegister(1).value().RegisterValue), 1.5f);
  ASSERT_TRUE(driver.writeNumericRegister(1, 2.5f).ok());
  EXPECT_EQ(driver.readDigitalInputPort(1).value().PortValue, 1);
  ASSERT_TRUE(driver.writeDigitalOutputPort(1, true).ok());
  EXPECT_FLOAT_EQ(std::get<float>(driver.readIOPort("AI", 2).value().PortValue), 2.5f);
  ASSERT_TRUE(driver.writeIOPort(2, "AO", 3.5f).ok());
  EXPECT_EQ(std::get<int>(driver.readVariablePacket("$TEST").data.value().VariableValue), 7);
  ASSERT_TRUE(driver.writeVariablePacket("$TEST", 8).ok());
  ASSERT_NO_THROW(controller.finish());
  EXPECT_NE(controller.requests()[1].find(R"("RegisterValue":2.5)"), std::string::npos);
  EXPECT_NE(controller.requests()[7].find(R"("VariableValue":8)"), std::string::npos);
}

TEST(FanucDriver, ExpectedReadsPreserveErrorsAndZeroValues)
{
  Controller controller({
      { "FRC_ReadDIN", R"({"Command":"FRC_ReadDIN","ErrorID":0,"PortNumber":1,"PortValue":0})" },
      { "FRC_ReadRegister", R"({"Command":"FRC_ReadRegister","ErrorID":0,"DataType":"Integer","RegisterValue":0})" },
      { "FRC_ReadDIN", R"({"Command":"FRC_ReadDIN","ErrorID":2556966})" },
      { "FRC_ReadIOPort", R"({"Command":"FRC_ReadIOPort","ErrorID":42})" },
      { "FRC_ReadRegister", R"({"Command":"FRC_ReadRegister","ErrorID":2556942})" },
      { "FRC_ReadPositionRegister", R"({"Command":"FRC_ReadPositionRegister","ErrorID":2556932})" },
      { "FRC_ReadIOPort", "" },
      { "FRC_ReadDIN", "", 0ms, 0, true },
  });
  FanucRmiDriver fanuc("127.0.0.1", controller.port());
  Robot_driver& robot = fanuc;
  ASSERT_TRUE(robot.connect().ok());
  const auto off = robot.readDigitalInputPort(1);
  ASSERT_TRUE(off.has_value());
  EXPECT_FALSE(*off);
  const auto zero = fanuc.readNumericRegister(1);
  ASSERT_TRUE(zero.has_value());
  EXPECT_EQ(std::get<int>(zero->RegisterValue), 0);

  const auto input = robot.readDigitalInputPort(1);
  ASSERT_FALSE(input.has_value());
  EXPECT_EQ(input.error().code(), 9999);
  EXPECT_FALSE(input.error().detail.empty());
  const auto io = fanuc.readIOPort("AI", 1);
  ASSERT_FALSE(io.has_value());
  EXPECT_EQ(io.error().code(), 9999);
  EXPECT_NE(io.error().detail.find("Unknown Error Code: 42"), std::string::npos);
  const auto numeric = fanuc.readNumericRegister(1);
  ASSERT_FALSE(numeric.has_value());
  EXPECT_EQ(numeric.error().code(), 9999);
  const auto position = fanuc.readPositionRegister(1);
  ASSERT_FALSE(position.has_value());
  EXPECT_EQ(position.error().code(), 9999);
  const auto timeout = fanuc.readIOPort("AI", 1, 0.01);
  ASSERT_FALSE(timeout.has_value());
  EXPECT_EQ(timeout.error().value, ResponseCode::WAITING_TIMEOUT);
  const auto closed = robot.readDigitalInputPort(1);
  ASSERT_FALSE(closed.has_value());
  EXPECT_EQ(closed.error().value, ResponseCode::CONNECTION_ERROR);
  ASSERT_NO_THROW(controller.finish());
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
  FanucRmiDriver driver("127.0.0.1", controller.port());
  ASSERT_TRUE(driver.connect().ok());
  ASSERT_TRUE(driver.initializeRemoteMotion().ok());
  EXPECT_EQ(driver.getRemainingBufferSize(), 8);
  rmi::JointMotionJRepPacket::Request joint{};
  joint.SpeedType = "Percent";
  joint.Speed = 10;
  joint.TermType = "FINE";
  EXPECT_EQ(driver.sendJointMotion(joint).data.value().SequenceID, 1);
  rmi::LinearRelativePacket::Request linear{};
  linear.Position.Z = 10;
  linear.SpeedType = "mmSec";
  linear.Speed = 10;
  linear.TermType = "FINE";
  EXPECT_EQ(driver.sendRMIPacket<rmi::LinearRelativePacket>(linear).data.value().SequenceID, 2);
  rmi::WaitForTimePacket::Request wait{};
  driver.sendRMIPacketNonBlocking(wait);
  EXPECT_EQ(wait.SequenceID, 3);
  EXPECT_EQ(driver.programCall("TEST", 1.0, { { "Integer", 7 } }).data.value().SequenceID, 4);
  EXPECT_EQ(driver.getLastInstructionResponse()->SequenceID, 4);
  EXPECT_EQ(driver.getRemainingBufferSize(), 8);
  ASSERT_NO_THROW(controller.finish());
  EXPECT_NE(controller.requests()[4].find(R"("ParamValue1":7)"), std::string::npos);
}

TEST(FanucDriver, AsyncInstructionsPipelineAndRetainEveryResult)
{
  Controller controller({
      { "FRC_WaitTime", "" },
      { "FRC_WaitTime", "" },
      { "FRC_WaitTime", R"({"Instruction":"FRC_WaitTime","ErrorID":0,"SequenceID":3})"
                        "\r\n"
                        R"({"Instruction":"FRC_WaitTime","ErrorID":2556959,"SequenceID":1})" },
      { "FRC_Reset", R"({"Instruction":"FRC_WaitTime","ErrorID":0,"SequenceID":2})"
                     "\r\n"
                     R"({"Command":"FRC_Reset","ErrorID":0})" },
  });
  FanucRmiDriver driver("127.0.0.1", controller.port());
  ASSERT_TRUE(driver.connect().ok());
  const auto first = driver.sendInstructionAsync(rmi::WaitForTimePacket::Request{});
  const auto second = driver.sendInstructionAsync(rmi::WaitForTimePacket::Request{});
  const auto third = driver.sendInstructionAsync(rmi::WaitForTimePacket::Request{});
  EXPECT_EQ(first.sequence_id, 1);
  EXPECT_EQ(second.sequence_id, 2);
  EXPECT_EQ(third.sequence_id, 3);
  EXPECT_EQ(second.result.wait_for(10ms), std::future_status::timeout);
  ASSERT_TRUE(driver.reset().ok());
  ASSERT_EQ(first.result.wait_for(1s), std::future_status::ready);
  ASSERT_EQ(second.result.wait_for(1s), std::future_status::ready);
  ASSERT_EQ(third.result.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(first.result.get().status.value, ResponseCode::CUSTOM_ERROR);
  EXPECT_EQ(first.result.get().data->ErrorID, 2556959);
  EXPECT_TRUE(second.result.get().ok());
  EXPECT_TRUE(third.result.get().ok());
  EXPECT_EQ(first.result.get().data->SequenceID, 1);
  EXPECT_EQ(driver.getLastInstructionResponse()->SequenceID, 3);
  EXPECT_EQ(driver.getRemainingBufferSize(), 8);
  ASSERT_NO_THROW(controller.finish());
}

TEST(FanucDriver, AsyncWindowLimitsInstructionsButAllowsSynchronousCommands)
{
  std::vector<Exchange> exchanges(8, { "FRC_WaitTime", "" });
  std::string responses;
  for (int sequence = 8; sequence >= 1; --sequence)
    responses += R"({"Instruction":"FRC_WaitTime","ErrorID":0,"SequenceID":)" + std::to_string(sequence) + "}\r\n";
  exchanges.push_back({ "FRC_Reset", responses + R"({"Command":"FRC_Reset","ErrorID":0})" });
  exchanges.push_back({ "FRC_WaitTime", R"({"Instruction":"FRC_WaitTime","ErrorID":0,"SequenceID":9})" });
  Controller controller(std::move(exchanges));
  FanucRmiDriver driver("127.0.0.1", controller.port());
  ASSERT_TRUE(driver.connect().ok());
  std::vector<FanucRmiDriver::InstructionHandle> handles;
  for (int i = 0; i < 9; ++i)
    handles.push_back(driver.sendInstructionAsync(rmi::WaitForTimePacket::Request{}));
  EXPECT_EQ(driver.getRemainingBufferSize(), 0);
  // The ninth motion must stay queued while the command bypasses the full window.
  ASSERT_TRUE(driver.reset().ok());
  for (const auto& handle : handles)
  {
    ASSERT_EQ(handle.result.wait_for(1s), std::future_status::ready);
    EXPECT_TRUE(handle.result.get().ok());
  }
  ASSERT_NO_THROW(controller.finish());
  EXPECT_NE(controller.requests()[8].find("FRC_Reset"), std::string::npos);
  EXPECT_NE(controller.requests()[9].find(R"("SequenceID":9)"), std::string::npos);
}

TEST(FanucDriver, AsyncInstructionsCompleteWithoutPollingTheDriver)
{
  Controller controller({
      { "FRC_WaitTime", R"({"Instruction":"FRC_WaitTime","ErrorID":0,"SequenceID":1})", 20ms },
  });
  FanucRmiDriver driver("127.0.0.1", controller.port());
  ASSERT_TRUE(driver.connect().ok());
  const auto handle = driver.sendInstructionAsync(rmi::WaitForTimePacket::Request{});
  ASSERT_EQ(handle.result.wait_for(1s), std::future_status::ready);
  EXPECT_TRUE(handle.result.get().ok());
  ASSERT_NO_THROW(controller.finish());
}

TEST(FanucDriver, AsyncWindowWakesWriterForEachReleasedSlot)
{
  std::vector<Exchange> exchanges(8, { "FRC_WaitTime", "" });
  exchanges.back().response = R"({"Instruction":"FRC_WaitTime","ErrorID":0,"SequenceID":1})";
  exchanges.back().delay = 20ms;
  exchanges.push_back({ R"("SequenceID":9)", R"({"Instruction":"FRC_WaitTime","ErrorID":0,"SequenceID":2})", 20ms });
  std::string remaining;
  for (int sequence = 3; sequence <= 10; ++sequence)
    remaining += R"({"Instruction":"FRC_WaitTime","ErrorID":0,"SequenceID":)" + std::to_string(sequence) + "}\r\n";
  exchanges.push_back({ R"("SequenceID":10)", std::move(remaining) });
  Controller controller(std::move(exchanges));
  FanucRmiDriver driver("127.0.0.1", controller.port());
  ASSERT_TRUE(driver.connect().ok());
  std::vector<FanucRmiDriver::InstructionHandle> handles;
  for (int i = 0; i < 10; ++i)
    handles.push_back(driver.sendInstructionAsync(rmi::WaitForTimePacket::Request{}));
  for (const auto& handle : handles)
  {
    ASSERT_EQ(handle.result.wait_for(1s), std::future_status::ready);
    EXPECT_TRUE(handle.result.get().ok());
  }
  EXPECT_EQ(driver.getRemainingBufferSize(), 8);
  ASSERT_NO_THROW(controller.finish());
  EXPECT_EQ(controller.requests().size(), 10);
}

TEST(FanucDriver, IdleSessionDestructionDrainsSuspendedCoroutines)
{
  Controller controller({ { "FRC_Reset", R"({"Command":"FRC_Reset","ErrorID":0})" } });
  auto driver = std::make_unique<FanucRmiDriver>("127.0.0.1", controller.port());
  ASSERT_TRUE(driver->connect().ok());
  ASSERT_TRUE(driver->reset().ok());
  const auto start = std::chrono::steady_clock::now();
  driver.reset();
  EXPECT_LT(std::chrono::steady_clock::now() - start, 1s);
  ASSERT_NO_THROW(controller.finish());
}

TEST(FanucDriver, AsyncInstructionsRejectMismatchedReplyNames)
{
  Controller controller({
      { "FRC_WaitTime", R"({"Instruction":"FRC_LinearMotion","ErrorID":0,"SequenceID":1})" },
  });
  FanucRmiDriver driver("127.0.0.1", controller.port());
  ASSERT_TRUE(driver.connect().ok());
  const auto handle = driver.sendInstructionAsync(rmi::WaitForTimePacket::Request{});
  ASSERT_EQ(handle.result.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(handle.result.get().status.value, ResponseCode::SERVER_ERR);
  EXPECT_FALSE(handle.result.get().data);
  ASSERT_NO_THROW(controller.finish());
}

TEST(FanucDriver, AsyncInstructionsReportUnknownInstructionReplies)
{
  Controller controller({
      { "FRC_WaitTime", R"({"Command":"Unknown","ErrorID":2556948,"SequenceID":1})" },
      { "FRC_WaitTime", R"({"Instruction":"FRC_WaitTime","ErrorID":0,"SequenceID":2})" },
  });
  FanucRmiDriver driver("127.0.0.1", controller.port());
  ASSERT_TRUE(driver.connect().ok());
  const auto rejected = driver.sendInstructionAsync(rmi::WaitForTimePacket::Request{});
  const auto accepted = driver.sendInstructionAsync(rmi::WaitForTimePacket::Request{});
  ASSERT_EQ(rejected.result.wait_for(1s), std::future_status::ready);
  ASSERT_EQ(accepted.result.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(rejected.result.get().status.value, ResponseCode::CUSTOM_ERROR);
  EXPECT_NE(rejected.result.get().status.detail.find("Invalid RMI Instruction"), std::string::npos);
  EXPECT_TRUE(accepted.result.get().ok());
  EXPECT_EQ(driver.getRemainingBufferSize(), 8);
  EXPECT_TRUE(driver.checkUnknownPacket());
  ASSERT_NO_THROW(controller.finish());
}

TEST(FanucDriver, SessionFaultsTerminatePendingAndQueuedInstructions)
{
  for (const auto& fault :
       { R"({"Communication":"FRC_SystemFault","SequenceID":8})", R"({"Communication":"FRC_Terminate"})" })
  {
    std::vector<Exchange> exchanges(8, { "FRC_WaitTime", "" });
    exchanges.back().response = fault;
    Controller controller(std::move(exchanges));
    FanucRmiDriver driver("127.0.0.1", controller.port());
    ASSERT_TRUE(driver.connect().ok());
    std::vector<FanucRmiDriver::InstructionHandle> handles;
    for (int i = 0; i < 9; ++i)
      handles.push_back(driver.sendInstructionAsync(rmi::WaitForTimePacket::Request{}));
    for (const auto& handle : handles)
    {
      ASSERT_EQ(handle.result.wait_for(1s), std::future_status::ready);
      EXPECT_EQ(handle.result.get().status.value, ResponseCode::CONTROLLER_ERROR);
    }
    ASSERT_NO_THROW(controller.finish());
    EXPECT_EQ(controller.requests().size(), 8);
  }
}

TEST(FanucDriver, FailedSessionSwitchDoesNotLeaveCallsWaitingForAnIoThread)
{
  Controller controller({}, {}, true);
  FanucRmiDriver driver("127.0.0.1", controller.port());
  EXPECT_EQ(driver.connect().status.value, ResponseCode::CONNECTION_ERROR);
  EXPECT_EQ(driver.reset().status.value, ResponseCode::CONNECTION_ERROR);
  EXPECT_EQ(driver.initializeRemoteMotion().status.value, ResponseCode::CONNECTION_ERROR);
  const auto handle = driver.sendInstructionAsync(rmi::WaitForTimePacket::Request{});
  ASSERT_EQ(handle.result.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(handle.result.get().status.value, ResponseCode::CONNECTION_ERROR);
  EXPECT_THROW(driver.getRemainingBufferSize(), std::runtime_error);
  ASSERT_NO_THROW(controller.finish());
}

TEST(FanucDriver, AsyncInstructionsReportDisconnectForEveryHandle)
{
  Controller controller({ { "FRC_WaitTime", "", 0ms, 0, true } });
  FanucRmiDriver driver("127.0.0.1", controller.port());
  ASSERT_TRUE(driver.connect().ok());
  std::vector<FanucRmiDriver::InstructionHandle> handles;
  for (int i = 0; i < 10; ++i)
    handles.push_back(driver.sendInstructionAsync(rmi::WaitForTimePacket::Request{}));
  for (const auto& handle : handles)
  {
    ASSERT_EQ(handle.result.wait_for(1s), std::future_status::ready);
    EXPECT_EQ(handle.result.get().status.value, ResponseCode::CONNECTION_ERROR);
  }
  ASSERT_NO_THROW(controller.finish());
}

TEST(FanucDriver, AsyncHandlesSurviveDriverDestruction)
{
  Controller controller(std::vector<Exchange>(8, { "FRC_WaitTime", "" }));
  std::vector<FanucRmiDriver::InstructionHandle> handles;
  {
    FanucRmiDriver driver("127.0.0.1", controller.port());
    ASSERT_TRUE(driver.connect().ok());
    for (int i = 0; i < 9; ++i)
      handles.push_back(driver.sendInstructionAsync(rmi::WaitForTimePacket::Request{}));
    ASSERT_TRUE(controller.waitForRequests(8));
  }
  for (const auto& handle : handles)
  {
    ASSERT_EQ(handle.result.wait_for(1s), std::future_status::ready);
    EXPECT_EQ(handle.result.get().status.value, ResponseCode::CONNECTION_ERROR);
  }
  ASSERT_NO_THROW(controller.finish());
}

TEST(FanucDriver, SynchronousMotionDoesNotBlockAsyncSubmissionsOrCommands)
{
  Controller controller({
      { "FRC_WaitTime", "" },
      { "FRC_Reset", R"({"Command":"FRC_Reset","ErrorID":0})" },
      { "FRC_WaitTime", R"({"Instruction":"FRC_WaitTime","ErrorID":0,"SequenceID":2})"
                        "\r\n"
                        R"({"Instruction":"FRC_WaitTime","ErrorID":0,"SequenceID":1})" },
  });
  FanucRmiDriver driver("127.0.0.1", controller.port());
  ASSERT_TRUE(driver.connect().ok());
  auto synchronous = std::async(std::launch::async, [&]() {
    rmi::WaitForTimePacket::Request request{};
    return driver.sendRMIPacket<rmi::WaitForTimePacket>(request);
  });
  ASSERT_TRUE(controller.waitForRequests(1));
  ASSERT_TRUE(driver.reset().ok());
  const auto asynchronous = driver.sendInstructionAsync(rmi::WaitForTimePacket::Request{});
  ASSERT_EQ(synchronous.wait_for(1s), std::future_status::ready);
  EXPECT_TRUE(synchronous.get().ok());
  ASSERT_EQ(asynchronous.result.wait_for(1s), std::future_status::ready);
  EXPECT_TRUE(asynchronous.result.get().ok());
  ASSERT_NO_THROW(controller.finish());
}

TEST(FanucDriver, InitializeRejectsOutstandingInstructionsAndResetsAfterAbort)
{
  Controller controller({
      { "FRC_WaitTime", "" },
      { "FRC_Abort", R"({"Command":"FRC_Abort","ErrorID":0})" },
      { "FRC_Initialize", R"({"Command":"FRC_Initialize","ErrorID":0})" },
      { "FRC_WaitTime", R"({"Instruction":"FRC_WaitTime","ErrorID":0,"SequenceID":1})" },
  });
  FanucRmiDriver driver("127.0.0.1", controller.port());
  ASSERT_TRUE(driver.connect().ok());
  const auto old = driver.sendInstructionAsync(rmi::WaitForTimePacket::Request{});
  EXPECT_EQ(driver.initializeRemoteMotion().status.value, ResponseCode::INVALID_PARAMETER);
  ASSERT_TRUE(driver.abort().ok());
  ASSERT_TRUE(driver.initializeRemoteMotion().ok());
  const auto next = driver.sendInstructionAsync(rmi::WaitForTimePacket::Request{});
  EXPECT_EQ(next.sequence_id, 1);
  ASSERT_EQ(next.result.wait_for(1s), std::future_status::ready);
  ASSERT_EQ(old.result.wait_for(1s), std::future_status::ready);
  EXPECT_TRUE(next.result.get().ok());
  EXPECT_EQ(old.result.get().status.value, ResponseCode::CONTROLLER_ERROR);
  ASSERT_NO_THROW(controller.finish());
}

TEST(FanucDriver, AbortTerminatesPendingAndQueuedInstructions)
{
  std::vector<Exchange> exchanges(8, { "FRC_WaitTime", "" });
  exchanges.push_back({ "FRC_Abort", R"({"Command":"FRC_Abort","ErrorID":0})" });
  Controller controller(std::move(exchanges));
  FanucRmiDriver driver("127.0.0.1", controller.port());
  ASSERT_TRUE(driver.connect().ok());
  std::vector<FanucRmiDriver::InstructionHandle> handles;
  for (int i = 0; i < 9; ++i)
    handles.push_back(driver.sendInstructionAsync(rmi::WaitForTimePacket::Request{}));
  ASSERT_TRUE(driver.abort().ok());
  for (const auto& handle : handles)
  {
    ASSERT_EQ(handle.result.wait_for(1s), std::future_status::ready);
    EXPECT_EQ(handle.result.get().status.value, ResponseCode::CONTROLLER_ERROR);
  }
  const auto rejected = driver.sendInstructionAsync(rmi::WaitForTimePacket::Request{});
  ASSERT_EQ(rejected.result.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(rejected.result.get().status.value, ResponseCode::CONTROLLER_ERROR);
  ASSERT_NO_THROW(controller.finish());
  EXPECT_EQ(controller.requests().size(), 9);
}

TEST(FanucDriver, SynchronousInstructionTimeoutDoesNotLoseItsReply)
{
  Controller controller({
      { "FRC_WaitTime", "" },
      { "FRC_WaitTime", R"({"Instruction":"FRC_WaitTime","ErrorID":0,"SequenceID":2})"
                        "\r\n"
                        R"({"Instruction":"FRC_WaitTime","ErrorID":0,"SequenceID":1})" },
  });
  FanucRmiDriver driver("127.0.0.1", controller.port());
  ASSERT_TRUE(driver.connect().ok());
  rmi::WaitForTimePacket::Request request{};
  EXPECT_EQ(driver.sendRMIPacket<rmi::WaitForTimePacket>(request, 0.01).status.value, ResponseCode::WAITING_TIMEOUT);
  const auto next = driver.sendInstructionAsync(request);
  ASSERT_EQ(next.result.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(next.result.get().data->SequenceID, 2);
  EXPECT_TRUE(next.result.get().ok());
  ASSERT_NO_THROW(controller.finish());
}

TEST(FanucDriver, ControllerErrorsAndTimeoutsAreReported)
{
  Controller controller({
      { "FRC_Reset", R"({"Command":"FRC_Reset","ErrorID":2556940})" },
      { "FRC_GetStatus", "" },
  });
  FanucRmiDriver driver("127.0.0.1", controller.port());
  ASSERT_TRUE(driver.connect().ok());
  const auto reset = driver.reset();
  EXPECT_FALSE(reset.ok());
  EXPECT_FALSE(reset.data);
  EXPECT_EQ(reset.status.code(), 9999);
  EXPECT_NE(reset.status.detail.find("Cannot Reset Controller"), std::string::npos);
  const auto status = driver.getStatus(0.01);
  EXPECT_EQ(status.status.code(), 4411);
  EXPECT_FALSE(status.data);
  ASSERT_NO_THROW(controller.finish());
}

TEST(FanucDriver, FragmentedAndLargeResponses)
{
  const std::string alarm(8192, 'a');
  Controller controller({
      { "FRC_Reset", R"({"Command":"FRC_Reset","ErrorID":0})", 0ms, 1 },
      { "FRC_ReadError", R"({"Command":"FRC_ReadError","ErrorID":0,"ErrorData":")" + alarm + R"("})" },
  });
  FanucRmiDriver driver("localhost", controller.port());
  ASSERT_TRUE(driver.connect().ok());
  EXPECT_EQ(driver.reset().data.value().ErrorID, 0);
  EXPECT_EQ(driver.readError().data.value().ErrorData, alarm);
  ASSERT_NO_THROW(controller.finish());
}

TEST(FanucDriver, ForwardedAndTemplateCallsPreserveControllerErrors)
{
  Controller controller({
      { "FRC_ReadError", R"({"Command":"FRC_ReadError","ErrorID":42})" },
      { "FRC_Call", R"({"Instruction":"FRC_Call","ErrorID":2556940,"SequenceID":1})" },
      { "FRC_ReadUFrameData", R"({"Command":"FRC_ReadUFrameData","ErrorID":2556931})" },
      { "FRC_JointMotionJRep", R"({"Instruction":"FRC_JointMotionJRep","ErrorID":2556959,"SequenceID":2})" },
  });
  FanucRmiDriver driver("127.0.0.1", controller.port());
  ASSERT_TRUE(driver.connect().ok());
  const auto error = driver.readError();
  EXPECT_EQ(error.status.code(), 9999);
  EXPECT_NE(error.status.detail.find("Unknown Error Code: 42"), std::string::npos);
  EXPECT_FALSE(error.data);
  const auto program = driver.programCall("TEST");
  EXPECT_EQ(program.status.code(), 9999);
  EXPECT_FALSE(program.data);
  const auto frame = driver.readUFrameData(1);
  EXPECT_EQ(frame.status.code(), 9999);
  EXPECT_FALSE(frame.data);
  rmi::JointMotionJRepPacket::Request request{};
  const auto motion = driver.sendRMIPacket<rmi::JointMotionJRepPacket>(request);
  EXPECT_EQ(motion.status.code(), 9999);
  EXPECT_FALSE(motion.data);
  ASSERT_NO_THROW(controller.finish());
}

TEST(FanucDriver, InvalidParametersReturnStatusWithoutSendingCommands)
{
  Controller controller({ { "FRC_Pause", R"({"Command":"FRC_Pause","ErrorID":0})" } });
  FanucRmiDriver driver("127.0.0.1", controller.port());
  ASSERT_TRUE(driver.connect().ok());
  const auto check = [](const auto& response) {
    EXPECT_EQ(response.status.value, ResponseCode::INDEX_OUT_OF_RANGE);
    EXPECT_FALSE(response.data);
    EXPECT_FALSE(response.status.detail.empty());
  };
  const auto check_read = [](const auto& response) {
    ASSERT_FALSE(response.has_value());
    EXPECT_EQ(response.error().value, ResponseCode::INDEX_OUT_OF_RANGE);
    EXPECT_FALSE(response.error().detail.empty());
  };
  for (int number : { 0, 101 })
  {
    check_read(driver.readPositionRegister(number));
    check(driver.writePositionRegister(number, "Joint", {}, {}, {}));
  }
  for (int number : { 0, 201 })
  {
    check_read(driver.readNumericRegister(number));
    check(driver.writeNumericRegister(number, 1));
  }
  const std::vector<rmi::RMICallParam> params(11, { "Integer", 1 });
  const auto program = driver.programCall("TEST", 1.0, params);
  EXPECT_EQ(program.status.value, ResponseCode::INVALID_PARAMETER);
  EXPECT_FALSE(program.data);
  ASSERT_TRUE(driver.pause().ok());
  ASSERT_NO_THROW(controller.finish());
  EXPECT_EQ(controller.requests().size(), 1);
}

TEST(FanucDriver, NewSessionDiscardsHandshakeFragment)
{
  Controller controller({ { "FRC_Reset", R"({"Command":"FRC_Reset","ErrorID":0})" } }, R"({"stale":)");
  FanucRmiDriver driver("127.0.0.1", controller.port());
  EXPECT_EQ(driver.connect().data.value().MajorVersion, 3);
  EXPECT_EQ(driver.reset().data.value().ErrorID, 0);
  ASSERT_NO_THROW(controller.finish());
}

TEST(FanucDriver, ResponseTimeoutPreservesPendingRead)
{
  Controller controller({
      { "FRC_Reset", R"({"Command":"FRC_Reset","ErrorID":0})", 100ms },
      { "FRC_Pause", R"({"Command":"FRC_Pause","ErrorID":0})" },
  });
  FanucRmiDriver driver("127.0.0.1", controller.port());
  ASSERT_TRUE(driver.connect().ok());
  const auto start = std::chrono::steady_clock::now();
  const auto response = driver.reset(0.01);
  EXPECT_EQ(response.status.value, ResponseCode::WAITING_TIMEOUT);
  EXPECT_FALSE(response.data);
  EXPECT_LT(std::chrono::steady_clock::now() - start, 80ms);
  EXPECT_EQ(driver.pause().data.value().ErrorID, 0);
  ASSERT_NO_THROW(controller.finish());
}

TEST(FanucDriver, UnlimitedAndLargeTimeoutsWaitForData)
{
  Controller controller({
      { "FRC_Reset", R"({"Command":"FRC_Reset","ErrorID":0})", 20ms },
      { "FRC_Pause", R"({"Command":"FRC_Pause","ErrorID":0})", 20ms },
  });
  FanucRmiDriver driver("127.0.0.1", controller.port());
  ASSERT_TRUE(driver.connect().ok());
  EXPECT_EQ(driver.reset(-1.0).data.value().ErrorID, 0);
  EXPECT_EQ(driver.pause(std::numeric_limits<double>::max()).data.value().ErrorID, 0);
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
  FanucRmiDriver driver("127.0.0.1", controller.port());
  ASSERT_TRUE(driver.connect().ok());
  for (const auto timeout : { std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
                              -std::numeric_limits<double>::infinity() })
  {
    const auto response = driver.reset(timeout);
    EXPECT_EQ(response.status.value, ResponseCode::INVALID_PARAMETER);
    EXPECT_FALSE(response.data);
    EXPECT_NE(response.status.detail.find("finite"), std::string::npos);
  }
  EXPECT_EQ(driver.pause().data.value().ErrorID, 0);
  ASSERT_NO_THROW(controller.finish());
}

TEST(FanucDriver, PushChecksDoNotWaitForData)
{
  Controller controller({ { "FRC_Reset", R"({"Command":"FRC_Reset","ErrorID":0})" } });
  FanucRmiDriver driver("127.0.0.1", controller.port());
  ASSERT_TRUE(driver.connect().ok());
  ASSERT_TRUE(driver.reset().ok());
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
  FanucRmiDriver driver("127.0.0.1", controller.port());
  ASSERT_TRUE(driver.connect().ok());
  EXPECT_EQ(driver.reset().data.value().ErrorID, 0);
  ASSERT_NO_THROW(controller.finish());
  EXPECT_THROW(driver.checkSystemFault(), std::runtime_error);
}

TEST(FanucDriver, EofWithoutCompleteResponseIsReported)
{
  Controller controller({ { "FRC_Reset", R"({"Command":)", 0ms, 0, true, false } });
  FanucRmiDriver driver("127.0.0.1", controller.port());
  ASSERT_TRUE(driver.connect().ok());
  const auto response = driver.reset();
  EXPECT_EQ(response.status.value, ResponseCode::CONNECTION_ERROR);
  EXPECT_FALSE(response.data);
  EXPECT_NE(response.status.detail.find("closed by the controller"), std::string::npos);
  ASSERT_NO_THROW(controller.finish());
}

TEST(FanucDriver, RefusedConnectionIsReported)
{
  boost::asio::io_context context;
  boost::asio::ip::tcp::acceptor listener(context, { boost::asio::ip::make_address("127.0.0.1"), 0 });
  const auto port = listener.local_endpoint().port();
  listener.close();
  EXPECT_THROW(FanucRmiDriver("127.0.0.1", port), std::runtime_error);
}

TEST(FanucDriver, ProgramCallMapsTenArgumentsAndRejectsOverflow)
{
  Controller controller({
      { "FRC_Call", R"({"Instruction":"FRC_Call","ErrorID":0,"SequenceID":1})" },
      { "FRC_Call", R"({"Instruction":"FRC_Call","ErrorID":0,"SequenceID":2})" },
  });
  FanucRmiDriver driver("127.0.0.1", controller.port());
  ASSERT_TRUE(driver.connect().ok());
  std::vector<rmi::RMICallParam> params;
  for (int i = 1; i <= 10; ++i)
    params.emplace_back("Integer", i);
  EXPECT_EQ(driver.programCall("TEST", 1.0, params).data.value().SequenceID, 1);
  params.emplace_back("Integer", 11);
  EXPECT_THROW(driver.programCallNonBlocking("TEST", params), std::invalid_argument);
  EXPECT_EQ(driver.programCall("NO_ARGS").data.value().SequenceID, 2);
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
  FanucRmiDriver driver("127.0.0.1", controller.port());
  ASSERT_TRUE(driver.connect().ok());
  EXPECT_EQ(driver.getUFrameUTool().data.value().UFrameNumber, 2);
  ASSERT_TRUE(driver.setUFrameUTool(2, 1, 1.0, 1).ok());
  EXPECT_FLOAT_EQ(driver.readUFrameData(2).data.value().Frame.X, 10);
  rmi::FrameData data{};
  data.X = 11;
  ASSERT_TRUE(driver.writeUFrameData(2, data).ok());
  EXPECT_FLOAT_EQ(driver.readUToolData(1).data.value().Frame.X, 10);
  ASSERT_TRUE(driver.writeUToolData(1, data).ok());
  EXPECT_FLOAT_EQ(driver.readJointAngles(1).data.value().JointAngle.J1, 15);
  EXPECT_FLOAT_EQ(driver.getCartesianPosition().data.value().Position.X, 20);
  EXPECT_FLOAT_EQ(driver.getTCPSpeed().data.value().Speed, 10);
  EXPECT_FLOAT_EQ(driver.readPositionRegister(1).value().JointAngle->J1, 15);
  rmi::JointAngleData target{};
  target.J1 = 30;
  ASSERT_TRUE(driver.writePositionRegister(1, "Joint", {}, {}, target).ok());
  EXPECT_EQ(driver.getPayloadSchedule().data.value().ScheduleNumber, 1);
  ASSERT_TRUE(driver.setPayloadSchedule(1).ok());
  EXPECT_FLOAT_EQ(driver.getPayloadValue(1).data.value().Mass, 2);
  ASSERT_TRUE(driver.setPayloadValue(1, 2, 0, 0, 0, true, 1, 2, 3).ok());
  EXPECT_FLOAT_EQ(driver.getPayloadComp(1).data.value().Mass, 3);
  ASSERT_TRUE(driver.setPayloadComp(1, 3, 0, 0, 0, 1, 2, 3).ok());
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
  FanucRmiDriver driver("127.0.0.1", controller.port());
  ASSERT_TRUE(driver.connect().ok());
  EXPECT_EQ(driver.readError(1.0, 2).data.value().ErrorData2, "detail");
  EXPECT_EQ(driver.getExtendedStatus().data.value().DrivesPowered, 1);
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
  EXPECT_EQ(driver.readError().data.value().ErrorData, "\xe3\x83\x86\xe3\x82\xb9\xe3\x83\x88");
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
  FanucRmiDriver driver("127.0.0.1", controller.port());
  ASSERT_TRUE(driver.connect().ok());
  ASSERT_TRUE(driver.initializeRemoteMotion().ok());
  EXPECT_EQ(driver.sendRMIPacket<TypeParam>(request).data.value().SequenceID, 1);
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
