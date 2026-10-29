// SPDX-FileCopyrightText: 2025-2026, FANUC America Corporation
// SPDX-FileCopyrightText: 2025-2026, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <thread>
#include <type_traits>

#include "fanuc_client/fanuc_client.hpp"
#include "fanuc_driver/fanuc_driver.hpp"

class MockStreamMotionConnection : public stream_motion::StreamMotionInterface
{
public:
  explicit MockStreamMotionConnection(std::atomic<bool>& stream_connected) : stream_connected_{ stream_connected }
  {
    status_.status = 1;
    status_.joint_angle = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  }

  bool streamConnected() const
  {
    return stream_connected_;
  }

  void sendStartPacket() override
  {
    stream_connected_ = true;
  }

  void sendStopPacket() const override
  {
    stream_connected_ = false;
  }

  MOCK_METHOD(void, clearRecvBuffer, (), (override));

  void sendCommand(const std::array<double, stream_motion::kMaxAxisNumber>& command_pos, bool is_last_command,
                   const std::array<uint8_t, 256>& io_command, const uint8_t do_motn_ctrl) const override
  {
    for (int i = 0; i < stream_motion::kMaxAxisNumber; ++i)
    {
      status_.joint_angle[i] = static_cast<float>(command_pos[i]);
    }
    status_.io_status = io_command;
  }

  bool getStatusPacket(stream_motion::RobotStatusPacket& status) override
  {
    if (!stream_connected_)
    {
      return false;
    }
    status = status_;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    return true;
  }

  bool getRobotLimits(const uint32_t axis_number, stream_motion::RobotThresholdPacket& robot_threshold_velocity,
                      stream_motion::RobotThresholdPacket& robot_threshold_acceleration,
                      stream_motion::RobotThresholdPacket& robot_threshold_jerk) const override
  {
    robot_threshold_velocity.axis_number = axis_number;
    robot_threshold_acceleration.axis_number = axis_number;
    robot_threshold_jerk.axis_number = axis_number;
    for (int i = 0; i < 20; ++i)
    {
      robot_threshold_velocity.full_payload[i] = 200.0f;
      robot_threshold_velocity.no_payload[i] = 200.0f;
      robot_threshold_acceleration.full_payload[i] = 2000.0f;
      robot_threshold_acceleration.no_payload[i] = 2000.0f;
      robot_threshold_jerk.full_payload[i] = 20000.0f;
      robot_threshold_jerk.no_payload[i] = 20000.0f;
    }
    return true;
  }

  bool configureGPIO(const stream_motion::GPIOConfiguration& config) const override
  {
    configured_gpio = config;
    return gpio_config_success;
  }

  bool getControllerCapability(stream_motion::ControllerCapabilityResultPacket& controller_capability) override
  {
    controller_capability.sampling_rate = sampling_rate;
    controller_capability.available_version = 3;
    return capability_success;
  }

  void configureForceSensor(uint32_t do_reset, uint32_t force_sensor_type) const override
  {
    if (!force_sensor_success)
    {
      throw std::runtime_error("Simulated force sensor setup failure");
    }
  }

  mutable stream_motion::GPIOConfiguration configured_gpio{};
  bool gpio_config_success = true;
  bool capability_success = true;
  bool force_sensor_success = true;
  uint32_t sampling_rate = 8;

private:
  mutable stream_motion::RobotStatusPacket status_;
  std::atomic<bool>& stream_connected_;
};

class MockRMIConnection : public rmi::RMIConnectionInterface
{
public:
  ~MockRMIConnection() override = default;

  MOCK_METHOD(rmi::ConnectROS2Packet::Response, connect, (std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::DisconnectPacket::Response, disconnect, (std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::InitializePacket::Response, initializeRemoteMotion, (std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::InitializePacket::Response, initializeRemoteMotion,
              (std::optional<double> timeout, const std::optional<uint8_t> groupmask,
               const std::optional<std::string>& rtsa, const std::optional<std::string>& pltzmode),
              (override));
  MOCK_METHOD(rmi::ProgramCallPacket::Response, programCall,
              (const std::string& program_name, std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::ProgramCallPacket::Response, programCall,
              (const std::string& program_name, std::optional<double> timeout,
               const std::vector<rmi::RMICallParam>& params),
              (override));
  MOCK_METHOD(rmi::ProgramCallPacket::Request, programCallNonBlocking, (const std::string& program_name), (override));
  MOCK_METHOD(rmi::ProgramCallPacket::Request, programCallNonBlocking,
              (const std::string& program_name, const std::vector<rmi::RMICallParam>& params), (override));
  MOCK_METHOD(rmi::StatusRequestPacket::Response, getStatus, (std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::SetSpeedOverridePacket::Response, setSpeedOverride, (int value, std::optional<double> timeout),
              (override));
  MOCK_METHOD(rmi::AbortPacket::Response, abort, (std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::PausePacket::Response, pause, (std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::ContinuePacket::Response, resume, (std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::ResetRobotPacket::Response, reset, (std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::ReadErrorPacket::Response, readError, (std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::ReadErrorPacket::Response, readError,
              (std::optional<double> timeout, const std::optional<uint8_t> count), (override));
  MOCK_METHOD(rmi::GetUFrameToolFramePacket::Response, getUFrameUTool,
              (std::optional<double> timeout, const std::optional<uint8_t> group), (override));
  MOCK_METHOD(rmi::SetUFrameToolFramePacket::Response, setUFrameUTool,
              (const int uframe, const int utool, std::optional<double> timeout, const std::optional<uint8_t> group),
              (override));
  MOCK_METHOD(rmi::ReadUFrameDataPacket::Response, readUFrameData,
              (const int uframe, std::optional<double> timeout, const std::optional<uint8_t> group), (override));
  MOCK_METHOD(rmi::WriteUFrameDataPacket::Response, writeUFrameData,
              (const int uframe, const rmi::FrameData data, std::optional<double> timeout,
               const std::optional<uint8_t> group),
              (override));
  MOCK_METHOD(rmi::ReadUToolDataPacket::Response, readUToolData,
              (const int utool, std::optional<double> timeout, const std::optional<uint8_t> group), (override));
  MOCK_METHOD(rmi::WriteUToolDataPacket::Response, writeUToolData,
              (const int utool, const rmi::FrameData data, std::optional<double> timeout,
               const std::optional<uint8_t> group),
              (override));
  MOCK_METHOD(rmi::WritePositionRegisterPacket::Response, writePositionRegister,
              (int register_number, const std::string& representation, const rmi::ConfigurationData& configuration,
               const rmi::PositionData& position, const rmi::JointAngleData& joint_angle, std::optional<double> timeout),
              (override));
  MOCK_METHOD(rmi::ReadPositionRegisterPacket::Response, readPositionRegister,
              (int register_number, std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::ReadNumericRegisterPacket::Response, readNumericRegister,
              (int register_number, std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::WriteNumericRegisterPacket::Response, writeNumericRegister,
              (int register_number, (std::variant<int, float>)value, std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::ReadDigitalInputPortPacket::Response, readDigitalInputPort,
              (uint16_t port_number, std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::WriteDigitalOutputPacket::Response, writeDigitalOutputPort,
              (uint16_t port_number, bool port_value, std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::ReadIOPortPacket::Response, readIOPort,
              (const std::string& port_type, int port_number, std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::WriteIOPortPacket::Response, writeIOPort,
              (int port_number, const std::string& port_type, (std::variant<int, float>)port_value,
               std::optional<double> timeout),
              (override));
  rmi::ReadVariablePacket::Response readVariablePacket(const std::string& variable_name,
                                                       std::optional<double> timeout) override
  {
    assert(variable_name == std::string("$STMO.$COM_INT"));
    rmi::ReadVariablePacket::Response response{};
    response.VariableValue = 8;
    return response;
  }
  MOCK_METHOD(rmi::WriteVariablePacket::Response, writeVariablePacket,
              (const std::string& variable_name, (std::variant<int, float>)value, std::optional<double> timeout),
              (override));
  MOCK_METHOD(rmi::GetExtendedStatusPacket::Response, getExtendedStatus, (std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::SetPayloadPacket::Response, setPayloadSchedule,
              (uint8_t payload_schedule_number, std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::SetPayloadValuePacket::Response, setPayloadValue,
              (uint8_t payload_schedule_number, float mass, float cg_x, float cg_y, float cg_z, bool use_in, float in_x,
               float in_y, float in_z, std::optional<double> timeout),
              (override));
  MOCK_METHOD(rmi::SetPayloadCompPacket::Response, setPayloadComp,
              (uint8_t payload_schedule_number, float mass, float cg_x, float cg_y, float cg_z, float in_x, float in_y,
               float in_z, std::optional<double> timeout),
              (override));
  MOCK_METHOD(rmi::GetPayloadPacket::Response, getPayloadSchedule, (std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::GetPayloadValuePacket::Response, getPayloadValue,
              (uint16_t payload_schedule_number, std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::GetPayloadCompPacket::Response, getPayloadComp,
              (uint16_t payload_schedule_number, std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::JointMotionJRepPacket::Response, sendJointMotion,
              (rmi::JointMotionJRepPacket::Request joint_motion_request, const std::optional<double> timeout),
              (override));
  MOCK_METHOD(rmi::ReadJointAnglesPacket::Response, readJointAngles,
              (const std::optional<uint8_t>& group, const std::optional<double> timeout), (override));
  MOCK_METHOD(rmi::GetCartesianPositionPacket::Response, getCartesianPosition,
              (std::optional<double> timeout, const std::optional<uint8_t> group), (override));
  MOCK_METHOD(rmi::GetTCPSpeedPacket::Response, getTCPSpeed,
              (std::optional<double> timeout, const std::optional<uint8_t> group), (override));
  MOCK_METHOD(void, sendRMIPacketNonBlocking, (rmi::JointMotionPacket::Request & packet), (override));
  MOCK_METHOD(void, sendRMIPacketNonBlocking, (rmi::JointMotionJRepPacket::Request & packet), (override));
  MOCK_METHOD(void, sendRMIPacketNonBlocking, (rmi::JointRelativePacket::Request & packet), (override));
  MOCK_METHOD(void, sendRMIPacketNonBlocking, (rmi::JointRelativeJRepPacket::Request & packet), (override));
  MOCK_METHOD(void, sendRMIPacketNonBlocking, (rmi::LinearMotionPacket::Request & packet), (override));
  MOCK_METHOD(void, sendRMIPacketNonBlocking, (rmi::LinearMotionJRepPacket::Request & packet), (override));
  MOCK_METHOD(void, sendRMIPacketNonBlocking, (rmi::LinearRelativePacket::Request & packet), (override));
  MOCK_METHOD(void, sendRMIPacketNonBlocking, (rmi::LinearRelativeJRepPacket::Request & packet), (override));
  MOCK_METHOD(void, sendRMIPacketNonBlocking, (rmi::CircularMotionPacket::Request & packet), (override));
  MOCK_METHOD(void, sendRMIPacketNonBlocking, (rmi::CircularRelativePacket::Request & packet), (override));
  MOCK_METHOD(void, sendRMIPacketNonBlocking, (rmi::SplineMotionPacket::Request & packet), (override));
  MOCK_METHOD(void, sendRMIPacketNonBlocking, (rmi::SplineMotionJRepPacket::Request & packet), (override));

  MOCK_METHOD(void, sendRMIPacketNonBlocking, (rmi::WaitForDINPacket::Request & packet), (override));
  MOCK_METHOD(void, sendRMIPacketNonBlocking, (rmi::SetUFramePacket::Request & packet), (override));
  MOCK_METHOD(void, sendRMIPacketNonBlocking, (rmi::SetToolFramePacket::Request & packet), (override));
  MOCK_METHOD(void, sendRMIPacketNonBlocking, (rmi::WaitForTimePacket::Request & packet), (override));
  MOCK_METHOD(void, sendRMIPacketNonBlocking, (rmi::SetPayloadInstructionPacket::Request & packet), (override));

  MOCK_METHOD(std::string, getErrorMessageString, (const uint32_t error_code), (override));

  MOCK_METHOD(int32_t, getRemainingBuffuerSize, (), (override));
  MOCK_METHOD(void, setEncoding, (const std::string& encoding), (override));

  MOCK_METHOD(std::optional<rmi::SystemFaultPacket>, checkSystemFault, (), (override));
  MOCK_METHOD(std::optional<rmi::TimeoutTerminatePacket>, checkTimeoutTerminate, (), (override));
  MOCK_METHOD(std::optional<rmi::CommunicationPacket>, checkCommunicationPacket, (), (override));
  MOCK_METHOD(std::optional<rmi::UnknownPacket>, checkUnknownPacket, (), (override));
  MOCK_METHOD(std::optional<rmi::InstructionResponse>, getLastInstructionResponse, (), (override));
};

using NiceMockStreamMotionConnection = testing::NiceMock<MockStreamMotionConnection>;
using NiceMockRMIConnection = testing::NiceMock<MockRMIConnection>;

TEST(FanucClientTest, TestSuccessfulLifecycle)
{
  std::atomic<bool> stream_connected = false;
  auto stream_motion_interface = std::make_unique<NiceMockStreamMotionConnection>(stream_connected);
  auto rmi_interface = std::make_unique<NiceMockRMIConnection>();
  fanuc_client::FanucClient fanuc_client("127.0.0.1", 60015, 16001, std::move(stream_motion_interface),
                                         std::move(rmi_interface), "UTF-8");

  // Reading/Writing data before starting the stream should throw an error
  const Eigen::VectorXd initial_joint_targets = Eigen::VectorXd::Zero(stream_motion::kMaxAxisNumber);
  EXPECT_THROW(fanuc_client.writeJointTarget(initial_joint_targets), std::runtime_error);
  EXPECT_THROW(fanuc_client.readJointAngles(), std::runtime_error);

  // Start the real-time stream with a mock connection
  fanuc_client.startRealtimeStream();
  while (!stream_connected)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_TRUE(stream_connected);

  EXPECT_THROW(fanuc_client.writeJointTarget(Eigen::VectorXd::Zero(stream_motion::kMaxAxisNumber - 1)),
               std::invalid_argument);

  // After starting the stream, we the first read should return the initial joint angles
  Eigen::VectorXd joint_states = fanuc_client.readJointAngles();
  EXPECT_EQ(joint_states, initial_joint_targets);

  // Writing joint targets should update the joint states eventually
  Eigen::VectorXd joint_targets = joint_states.array() + 1.0;
  while (joint_states == initial_joint_targets)
  {
    fanuc_client.writeJointTarget(joint_targets);
    joint_states = fanuc_client.readJointAngles();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_EQ(joint_targets, joint_states);

  // Stop the real-time stream should disconnect the stream
  fanuc_client.stopRealtimeStream();
  while (stream_connected)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_FALSE(stream_connected);
}

TEST(FanucClientTest, RejectsInvalidRMIJointTargetSize)
{
  std::atomic<bool> stream_connected = false;
  auto stream_motion_interface = std::make_unique<NiceMockStreamMotionConnection>(stream_connected);
  auto rmi_interface = std::make_unique<NiceMockRMIConnection>();
  fanuc_client::FanucClient fanuc_client("127.0.0.1", 60015, 16001, std::move(stream_motion_interface),
                                         std::move(rmi_interface), "UTF-8");

  EXPECT_THROW(fanuc_client.writeJointTargetRMI(Eigen::VectorXd::Zero(stream_motion::kMaxAxisNumber - 1)),
               std::invalid_argument);
}

TEST(FanucClientTest, TestGetLimits)
{
  std::atomic<bool> stream_connected = false;
  auto stream_motion_interface = std::make_unique<MockStreamMotionConnection>(stream_connected);
  auto rmi_interface = std::make_unique<NiceMockRMIConnection>();
  fanuc_client::FanucClient fanuc_client("127.0.0.1", 60015, 16001, std::move(stream_motion_interface),
                                         std::move(rmi_interface), "UTF-8");
  const double v_peak = 1000.0;
  const double payload = 0.0;
  std::vector<double> vel_limit;
  std::vector<double> acc_limit;
  std::vector<double> jerk_limit;
  fanuc_client.getLimits(v_peak, payload, vel_limit, acc_limit, jerk_limit);
  for (int i = 0; i < stream_motion::kMaxAxisNumber; ++i)
  {
    EXPECT_EQ(vel_limit[i], 200.0f);
    EXPECT_EQ(acc_limit[i], 2000.0f);
    EXPECT_EQ(jerk_limit[i], 20000.0f);
  }
}

TEST(RMISington, OnlyOneInstanceCreated)
{
  // Create the first MockRMIConnection instance and set it as the singleton
  auto rmi_inst = std::make_unique<MockRMIConnection>();
  fanuc_client::RMISingleton::setRMIInstance(std::move(rmi_inst));

  // Retrieve the singleton instance twice and check that both references point to the same object
  auto rmi_connection_inst1 = fanuc_client::RMISingleton::getRMIInstance();
  auto rmi_connection_inst2 = fanuc_client::RMISingleton::getRMIInstance();
  EXPECT_EQ(rmi_connection_inst1.get(), rmi_connection_inst2.get());

  // Create a new MockRMIConnection instance and replace the singleton
  auto new_rmi_inst = std::make_unique<MockRMIConnection>();
  MockRMIConnection* new_rmi_inst_ptr = new_rmi_inst.get();
  fanuc_client::RMISingleton::setRMIInstance(std::move(new_rmi_inst));

  // Retrieve the singleton instance again and check that it has changed to the new object
  auto rmi_connection_inst3 = fanuc_client::RMISingleton::getRMIInstance();
  EXPECT_NE(rmi_connection_inst2.get(), rmi_connection_inst3.get());
  EXPECT_EQ(rmi_connection_inst3.get(), new_rmi_inst_ptr);
}

static_assert(std::is_same_v<fanuc_client::FanucClient, fanuc_driver>);
static_assert(!std::is_copy_constructible_v<fanuc_driver>);
static_assert(!std::is_move_constructible_v<fanuc_driver>);
static_assert(std::is_nothrow_destructible_v<fanuc_driver>);

class FanucDriverTest : public testing::Test
{
protected:
  void SetUp() override
  {
    auto stream = std::make_unique<NiceMockStreamMotionConnection>(connected_);
    stream_ = stream.get();
    auto rmi = std::make_unique<NiceMockRMIConnection>();
    rmi_ = rmi.get();
    driver_ = std::make_unique<fanuc_driver>("127.0.0.1", 60015, 16001, std::move(stream), std::move(rmi));
  }

  std::atomic<bool> connected_ = false;
  NiceMockStreamMotionConnection* stream_;
  NiceMockRMIConnection* rmi_;
  std::unique_ptr<fanuc_driver> driver_;
};

TEST_F(FanucDriverTest, OwnsIndependentConnections)
{
  std::atomic<bool> second_connected = false;
  auto second_stream = std::make_unique<NiceMockStreamMotionConnection>(second_connected);
  auto second_rmi = std::make_unique<NiceMockRMIConnection>();
  auto* second_rmi_ptr = second_rmi.get();
  EXPECT_CALL(*rmi_, reset(testing::Eq(std::nullopt))).Times(1);
  EXPECT_CALL(*second_rmi_ptr, reset(testing::Eq(std::nullopt))).Times(1);
  {
    fanuc_driver second("127.0.0.2", 60016, 16002, std::move(second_stream), std::move(second_rmi));
    driver_->reset();
    second.reset();
  }
  // Destroying another driver must not replace or disconnect this driver's transport.
  EXPECT_CALL(*rmi_, writeDigitalOutputPort(7, true, testing::Eq(std::nullopt))).Times(1);
  driver_->writeDigitalOutputPort(7, true);
}

TEST_F(FanucDriverTest, ForwardsRMIArgumentsAndResponses)
{
  rmi::ReadNumericRegisterPacket::Response response{};
  response.RegisterValue = 12.5f;
  EXPECT_CALL(*rmi_, readNumericRegister(3, std::optional<double>{ 0.5 })).WillOnce(testing::Return(response));
  EXPECT_EQ(std::get<float>(driver_->readNumericRegister(3, 0.5).RegisterValue), 12.5f);
  EXPECT_CALL(*rmi_, writeIOPort(5, "AO", std::variant<int, float>{ 42 }, testing::Eq(std::nullopt))).Times(1);
  driver_->writeIOPort(5, "AO", 42);
  EXPECT_CALL(*rmi_, readNumericRegister(4, testing::Eq(std::nullopt)))
      .WillOnce(testing::Throw(std::runtime_error("RMI communication failed")));
  EXPECT_THROW(driver_->readNumericRegister(4), std::runtime_error);
}

TEST_F(FanucDriverTest, ConfiguresEveryGPIOType)
{
  gpio_config::GPIOConfig config;
  auto& topic = config.gpio_topic_config;
  topic.io_cmd = { { gpio_config::BoolIOCmdType::DO, 1, 2 },
                   { gpio_config::BoolIOCmdType::RO, 3, 4 },
                   { gpio_config::BoolIOCmdType::F, 5, 6 } };
  topic.io_state = { { gpio_config::BoolIOStateType::DI, 1, 2 },
                     { gpio_config::BoolIOStateType::DO, 3, 4 },
                     { gpio_config::BoolIOStateType::RI, 5, 6 },
                     { gpio_config::BoolIOStateType::RO, 7, 8 },
                     { gpio_config::BoolIOStateType::F, 9, 10 } };
  topic.analog_io_cmd = { { gpio_config::AnalogIOCmdType::AO, 1, 2 } };
  topic.analog_io_state = { { gpio_config::AnalogIOStateType::AI, 1, 2 }, { gpio_config::AnalogIOStateType::AO, 3, 4 } };
  topic.num_reg_cmd = { { 1, 2 } };
  topic.num_reg_state = { { 3, 4 } };
  driver_->configureGPIO(config);

  const auto& actual = stream_->configured_gpio;
  const std::array expected_types{ stream_motion::IOType::DO, stream_motion::IOType::RO, stream_motion::IOType::F,
                                   stream_motion::IOType::DI, stream_motion::IOType::DO, stream_motion::IOType::RI,
                                   stream_motion::IOType::RO, stream_motion::IOType::F,  stream_motion::IOType::AO,
                                   stream_motion::IOType::AI, stream_motion::IOType::AO };
  for (size_t i = 0; i < expected_types.size(); ++i)
  {
    EXPECT_EQ(actual[i].gpio_type, static_cast<uint32_t>(expected_types[i]));
  }
  EXPECT_EQ(actual[11].command_type, stream_motion::GPIOCommandType::NumRegCmd);
  EXPECT_EQ(actual[12].command_type, stream_motion::GPIOCommandType::NumRegState);
  EXPECT_EQ(actual[13].command_type, stream_motion::GPIOCommandType::None);
}

TEST_F(FanucDriverTest, PreservesConfiguredGPIOAcrossStreamRestarts)
{
  using Command = fanuc_driver::GPIOBuffer::CommandGPIOTypes;
  using State = fanuc_driver::GPIOBuffer::StatusGPIOTypes;
  gpio_config::GPIOConfig config;
  config.gpio_topic_config.io_cmd = { { gpio_config::BoolIOCmdType::DO, 1, 8 } };
  config.gpio_topic_config.io_state = { { gpio_config::BoolIOStateType::DO, 1, 8 } };
  config.gpio_topic_config.analog_io_cmd = { { gpio_config::AnalogIOCmdType::AO, 1, 1 } };
  config.gpio_topic_config.analog_io_state = { { gpio_config::AnalogIOStateType::AO, 1, 1 } };
  config.gpio_topic_config.num_reg_cmd = { { 1, 1 } };
  config.gpio_topic_config.num_reg_state = { { 1, 1 } };
  driver_->configureGPIO(config);

  for (int cycle = 0; cycle < 2; ++cycle)
  {
    const float expected_register = 42.5f + cycle;
    driver_->startRealtimeStream();
    driver_->writeGPIO<Command::DO, bool>(1, true);
    driver_->writeGPIO<Command::AO, uint16_t>(1, 1234);
    driver_->writeGPIO<Command::FloatReg, float>(1, expected_register);
    ASSERT_TRUE(driver_->sendIOCommand());
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (driver_->readGPIO<State::FloatReg, float>(1) != expected_register &&
           std::chrono::steady_clock::now() < deadline)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    EXPECT_TRUE((driver_->readGPIO<State::DO, bool>(1)));
    EXPECT_EQ((driver_->readGPIO<State::AO, uint16_t>(1)), 1234);
    EXPECT_FLOAT_EQ((driver_->readGPIO<State::FloatReg, float>(1)), expected_register);
    EXPECT_THROW(driver_->configureGPIO(config), std::logic_error);
    driver_->stopRealtimeStream();
    EXPECT_FALSE(driver_->isStreaming());
    EXPECT_NO_THROW(driver_->stopRealtimeStream());
  }
}

TEST_F(FanucDriverTest, RejectsInvalidGPIOAndPreservesPreviousConfiguration)
{
  using Command = fanuc_driver::GPIOBuffer::CommandGPIOTypes;
  EXPECT_THROW((driver_->writeGPIO<Command::DO, bool>(1, true)), std::logic_error);
  gpio_config::GPIOConfig config;
  config.gpio_topic_config.io_cmd = { { gpio_config::BoolIOCmdType::DO, 1, 8 } };
  driver_->configureGPIO(config);
  stream_->gpio_config_success = false;
  EXPECT_THROW(driver_->configureGPIO(config), std::runtime_error);
  EXPECT_NO_THROW((driver_->writeGPIO<Command::DO, bool>(1, true)));
  EXPECT_THROW((driver_->writeGPIO<Command::RO, bool>(1, true)), std::out_of_range);
  EXPECT_THROW((driver_->writeGPIO<Command::DO, bool>(0, true)), std::out_of_range);
  EXPECT_THROW((driver_->writeGPIO<Command::DO, bool>(9, true)), std::out_of_range);
  stream_->gpio_config_success = true;
  config.gpio_topic_config.io_cmd = { { gpio_config::BoolIOCmdType::DO, 1, 0 } };
  EXPECT_THROW(driver_->configureGPIO(config), std::invalid_argument);
  config.gpio_topic_config.io_cmd = { { gpio_config::BoolIOCmdType::DO, 1, UINT32_MAX } };
  EXPECT_THROW(driver_->configureGPIO(config), std::invalid_argument);
  config.gpio_topic_config.io_cmd = { { gpio_config::BoolIOCmdType::DO, 1, 8 },
                                      { gpio_config::BoolIOCmdType::DO, 4, 8 } };
  EXPECT_THROW(driver_->configureGPIO(config), std::invalid_argument);
}

TEST_F(FanucDriverTest, RejectsGPIOFailureBeforeStartingStream)
{
  fanuc_driver::GPIOBuffer::Builder builder;
  builder.addCommandConfig<fanuc_driver::GPIOBuffer::CommandGPIOTypes::DO, bool>(1, 8);
  auto buffer = std::make_shared<fanuc_driver::GPIOBuffer>(builder.build());
  stream_->gpio_config_success = false;
  EXPECT_THROW(driver_->startRealtimeStream(buffer), std::runtime_error);
  EXPECT_FALSE(connected_);
  EXPECT_FALSE(driver_->isStreaming());
}

TEST_F(FanucDriverTest, RollsBackFailedStartupAndCanRetry)
{
  stream_->force_sensor_success = false;
  EXPECT_CALL(*rmi_, initializeRemoteMotion(testing::_, testing::_, testing::_, testing::_)).Times(2);
  EXPECT_THROW(driver_->startRealtimeStream(), std::runtime_error);
  EXPECT_FALSE(driver_->isStreaming());
  EXPECT_FALSE(connected_);
  stream_->force_sensor_success = true;
  driver_->startRealtimeStream();
  EXPECT_TRUE(driver_->isStreaming());
  driver_->stopRealtimeStream();
}

TEST_F(FanucDriverTest, DestructionStopsAndJoinsActiveStream)
{
  EXPECT_CALL(*rmi_, disconnect(testing::Eq(std::nullopt))).Times(1);
  driver_->startRealtimeStream();
  driver_.reset();
  EXPECT_FALSE(connected_);
}

TEST_F(FanucDriverTest, LoadsGPIOFromYAMLThroughClassMethod)
{
  const std::filesystem::path root{ FANUC_DRIVER_SOURCE_DIR };
  driver_->loadGPIOConfig(root / "examples/gpio.yaml");
  EXPECT_EQ(stream_->configured_gpio[0].command_type, stream_motion::GPIOCommandType::IOCmd);
  EXPECT_EQ(stream_->configured_gpio[0].start, 1);
  EXPECT_EQ(stream_->configured_gpio[0].length, 8);
  EXPECT_EQ(stream_->configured_gpio[4].command_type, stream_motion::GPIOCommandType::NumRegCmd);
  EXPECT_EQ(stream_->configured_gpio[5].command_type, stream_motion::GPIOCommandType::NumRegState);
  EXPECT_THROW(driver_->loadGPIOConfig(root / "src/gpio_config/test/data/bad_config.yaml"), std::runtime_error);
  EXPECT_THROW(driver_->loadGPIOConfig(root / "examples/nonexistent.yaml"), std::runtime_error);
}

TEST_F(FanucDriverTest, GuardsRawRMIMotionDuringStreaming)
{
  driver_->startRealtimeStream();
  rmi::JointMotionJRepPacket::Request joint_request{};
  rmi::LinearMotionPacket::Request linear_request{};
  EXPECT_CALL(*rmi_, sendJointMotion(testing::_, testing::_)).Times(0);
  EXPECT_CALL(*rmi_, programCall(testing::_, testing::_, testing::_)).Times(0);
  EXPECT_THROW(driver_->sendJointMotion(joint_request), std::runtime_error);
  EXPECT_THROW(driver_->programCall("MOVE"), std::runtime_error);
  EXPECT_THROW(driver_->sendRMIPacketNonBlocking(linear_request), std::runtime_error);
  driver_->stopRealtimeStream();
  EXPECT_CALL(*rmi_, sendRMIPacketNonBlocking(testing::An<rmi::LinearMotionPacket::Request&>())).Times(1);
  driver_->sendRMIPacketNonBlocking(linear_request);
}

TEST(FanucDriverConstruction, DisconnectsOnCapabilityFailure)
{
  std::atomic<bool> connected = false;
  auto stream = std::make_unique<NiceMockStreamMotionConnection>(connected);
  stream->capability_success = false;
  auto rmi = std::make_unique<NiceMockRMIConnection>();
  EXPECT_CALL(*rmi, disconnect(testing::Eq(std::nullopt))).Times(1);
  EXPECT_THROW((fanuc_driver{ "127.0.0.1", 60015, 16001, std::move(stream), std::move(rmi) }), std::runtime_error);
}

TEST(FanucDriverConstruction, RejectsZeroControlPeriod)
{
  std::atomic<bool> connected = false;
  auto stream = std::make_unique<NiceMockStreamMotionConnection>(connected);
  stream->sampling_rate = 0;
  auto rmi = std::make_unique<NiceMockRMIConnection>();
  EXPECT_CALL(*rmi, disconnect(testing::Eq(std::nullopt))).Times(1);
  EXPECT_THROW((fanuc_driver{ "127.0.0.1", 60015, 16001, std::move(stream), std::move(rmi) }), std::runtime_error);
}

class FailingStreamMotionConnection : public NiceMockStreamMotionConnection
{
public:
  using NiceMockStreamMotionConnection::NiceMock;

  bool getStatusPacket(stream_motion::RobotStatusPacket& status) override
  {
    if (++status_calls_ > 1)
    {
      throw std::runtime_error("Simulated stream failure");
    }
    return MockStreamMotionConnection::getStatusPacket(status);
  }

private:
  int status_calls_ = 0;
};

TEST(FanucDriverLifecycle, PropagatesWorkerErrorsWithoutTerminating)
{
  std::atomic<bool> connected = false;
  auto stream = std::make_unique<FailingStreamMotionConnection>(connected);
  auto rmi = std::make_unique<NiceMockRMIConnection>();
  fanuc_driver driver("127.0.0.1", 60015, 16001, std::move(stream), std::move(rmi));
  driver.startRealtimeStream();
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (driver.isStreaming() && std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_FALSE(driver.isStreaming());
  EXPECT_THROW(driver.checkStreamError(), std::runtime_error);
  EXPECT_THROW(driver.readJointAngles(), std::runtime_error);
  EXPECT_THROW(driver.stopRealtimeStream(), std::runtime_error);
}
