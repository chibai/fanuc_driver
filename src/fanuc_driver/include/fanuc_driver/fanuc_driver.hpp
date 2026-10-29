// SPDX-FileCopyrightText: 2025-2026, FANUC America Corporation
// SPDX-FileCopyrightText: 2025-2026, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <concepts>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <variant>
#include <vector>

#include <Eigen/Core>

#include "fanuc_client/gpio_buffer.hpp"
#include "gpio_config/gpio_config.hpp"
#include "rmi/rmi.hpp"
#include "stream_motion/stream.hpp"

namespace fanuc_client
{

enum class ContactStopMode
{
  None = 0,
  SAFE = 1,
  STOP = 2,
  DSBL = 3,
  ESCP = 4,
};

struct RobotStatus
{
  bool in_error{};
  bool tp_enabled{};
  bool e_stopped{};
  bool motion_possible{};
  ContactStopMode contact_stop_mode = ContactStopMode::None;
  float safety_scale{};
};

struct ForceSensor
{
  float force_x{};
  float force_y{};
  float force_z{};
  float moment_x{};
  float moment_y{};
  float moment_z{};
  uint32_t fs_type{};
};

}  // namespace fanuc_client

class fanuc_driver final
{
public:
  using GPIOBuffer = fanuc_client::GPIOBuffer;
  using RobotStatus = fanuc_client::RobotStatus;
  using ForceSensor = fanuc_client::ForceSensor;
  using ContactStopMode = fanuc_client::ContactStopMode;
  using GPIOConfig = gpio_config::GPIOConfig;

  fanuc_driver() = delete;
  explicit fanuc_driver(std::string robot_ip, uint16_t stream_motion_port = 60015, uint16_t rmi_port = 16001,
                        std::unique_ptr<stream_motion::StreamMotionInterface> stream_motion_interface = nullptr,
                        std::unique_ptr<rmi::RMIConnectionInterface> rmi_connection_interface = nullptr,
                        std::string encoding = "");

  fanuc_driver(const fanuc_driver&) = delete;
  fanuc_driver& operator=(const fanuc_driver&) = delete;
  fanuc_driver(fanuc_driver&&) = delete;
  fanuc_driver& operator=(fanuc_driver&&) = delete;

  ~fanuc_driver() noexcept;

  void writeJointTarget(const Eigen::VectorXd& joint_targets);

  void writeJointTargetRMI(const Eigen::VectorXd& joint_targets);

  Eigen::Ref<const Eigen::VectorXd> readJointAngles();

  Eigen::Ref<const Eigen::VectorXd> readJointAnglesRMI();

  [[nodiscard]] bool sendIOCommand() const;

  void configureGPIO(const gpio_config::GPIOConfig& config);

  void loadGPIOConfig(const std::filesystem::path& path);

  template <GPIOBuffer::CommandGPIOTypes Type, typename T = GPIOBuffer::CommandValue<Type>>
    requires std::same_as<T, GPIOBuffer::CommandValue<Type>>
  void writeGPIO(int index, T value)
  {
    gpio_buffer().template commandBlock<Type, T>(index).set(index, value);
  }

  template <GPIOBuffer::StatusGPIOTypes Type, typename T = GPIOBuffer::StatusValue<Type>>
    requires std::same_as<T, GPIOBuffer::StatusValue<Type>>
  [[nodiscard]] T readGPIO(int index)
  {
    readStateFromQueue();
    return gpio_buffer().template statusBlock<Type, T>(index).get(index);
  }

  // Throws if it fails to start real-time communication
  void startRealtimeStream(std::shared_ptr<GPIOBuffer> gpio_buffer = nullptr);

  void stopRealtimeStream();

  void stopStreaming();

  [[nodiscard]] bool isStreaming() const noexcept;

  void checkStreamError() const;

  void startRMI();

  void abortRMI();

  [[nodiscard]] rmi::StatusRequestPacket::Response getStatus(std::optional<double> timeout = std::nullopt)
  {
    return rmi_connection_->getStatus(timeout);
  }

  rmi::ResetRobotPacket::Response reset(std::optional<double> timeout = std::nullopt)
  {
    return rmi_connection_->reset(timeout);
  }

  rmi::PausePacket::Response pause(std::optional<double> timeout = std::nullopt)
  {
    return rmi_connection_->pause(timeout);
  }

  rmi::ContinuePacket::Response resume(std::optional<double> timeout = std::nullopt)
  {
    return rmi_connection_->resume(timeout);
  }

  rmi::SetSpeedOverridePacket::Response setSpeedOverride(int value, std::optional<double> timeout = std::nullopt)
  {
    return rmi_connection_->setSpeedOverride(value, timeout);
  }

  [[nodiscard]] rmi::ReadErrorPacket::Response readError(std::optional<double> timeout = std::nullopt,
                                                         std::optional<uint8_t> count = std::nullopt)
  {
    return rmi_connection_->readError(timeout, count);
  }

  rmi::ProgramCallPacket::Response programCall(const std::string& name, std::optional<double> timeout = std::nullopt,
                                               const std::vector<rmi::RMICallParam>& params = {})
  {
    assertRMIMotionAllowed();
    return rmi_connection_->programCall(name, timeout, params);
  }

  rmi::ProgramCallPacket::Request programCallNonBlocking(const std::string& name,
                                                         const std::vector<rmi::RMICallParam>& params = {})
  {
    assertRMIMotionAllowed();
    return rmi_connection_->programCallNonBlocking(name, params);
  }

  rmi::JointMotionJRepPacket::Response sendJointMotion(rmi::JointMotionJRepPacket::Request request,
                                                       std::optional<double> timeout = std::nullopt)
  {
    assertRMIMotionAllowed();
    return rmi_connection_->sendJointMotion(std::move(request), timeout);
  }

  template <typename Packet>
    requires requires(rmi::RMIConnectionInterface& connection, Packet& packet) {
      connection.sendRMIPacketNonBlocking(packet);
    }
  void sendRMIPacketNonBlocking(Packet& packet)
  {
    assertRMIMotionAllowed();
    rmi_connection_->sendRMIPacketNonBlocking(packet);
  }

  [[nodiscard]] rmi::ReadNumericRegisterPacket::Response
  readNumericRegister(int number, std::optional<double> timeout = std::nullopt)
  {
    return rmi_connection_->readNumericRegister(number, timeout);
  }

  rmi::WriteNumericRegisterPacket::Response writeNumericRegister(int number, std::variant<int, float> value,
                                                                 std::optional<double> timeout = std::nullopt)
  {
    return rmi_connection_->writeNumericRegister(number, value, timeout);
  }

  [[nodiscard]] rmi::ReadPositionRegisterPacket::Response
  readPositionRegister(int number, std::optional<double> timeout = std::nullopt)
  {
    return rmi_connection_->readPositionRegister(number, timeout);
  }

  rmi::WritePositionRegisterPacket::Response writePositionRegister(int number, const std::string& representation,
                                                                   const rmi::ConfigurationData& configuration,
                                                                   const rmi::PositionData& position,
                                                                   const rmi::JointAngleData& joint_angle,
                                                                   std::optional<double> timeout = std::nullopt)
  {
    return rmi_connection_->writePositionRegister(number, representation, configuration, position, joint_angle, timeout);
  }

  [[nodiscard]] rmi::ReadDigitalInputPortPacket::Response
  readDigitalInputPort(uint16_t port, std::optional<double> timeout = std::nullopt)
  {
    return rmi_connection_->readDigitalInputPort(port, timeout);
  }

  rmi::WriteDigitalOutputPacket::Response writeDigitalOutputPort(uint16_t port, bool value,
                                                                 std::optional<double> timeout = std::nullopt)
  {
    return rmi_connection_->writeDigitalOutputPort(port, value, timeout);
  }

  [[nodiscard]] rmi::ReadIOPortPacket::Response readIOPort(const std::string& type, int port,
                                                           std::optional<double> timeout = std::nullopt)
  {
    return rmi_connection_->readIOPort(type, port, timeout);
  }

  rmi::WriteIOPortPacket::Response writeIOPort(int port, const std::string& type, std::variant<int, float> value,
                                               std::optional<double> timeout = std::nullopt)
  {
    return rmi_connection_->writeIOPort(port, type, value, timeout);
  }

  [[nodiscard]] rmi::ReadVariablePacket::Response readVariablePacket(const std::string& name,
                                                                     std::optional<double> timeout = std::nullopt)
  {
    return rmi_connection_->readVariablePacket(name, timeout);
  }

  rmi::WriteVariablePacket::Response writeVariablePacket(const std::string& name, std::variant<int, float> value,
                                                         std::optional<double> timeout = std::nullopt)
  {
    return rmi_connection_->writeVariablePacket(name, value, timeout);
  }

  [[nodiscard]] rmi::GetExtendedStatusPacket::Response getExtendedStatus(std::optional<double> timeout = std::nullopt)
  {
    return rmi_connection_->getExtendedStatus(timeout);
  }

  [[nodiscard]] rmi::GetUFrameToolFramePacket::Response getUFrameUTool(std::optional<double> timeout = std::nullopt,
                                                                       std::optional<uint8_t> group = std::nullopt)
  {
    return rmi_connection_->getUFrameUTool(timeout, group);
  }

  rmi::SetUFrameToolFramePacket::Response setUFrameUTool(int uframe, int utool,
                                                         std::optional<double> timeout = std::nullopt,
                                                         std::optional<uint8_t> group = std::nullopt)
  {
    return rmi_connection_->setUFrameUTool(uframe, utool, timeout, group);
  }

  [[nodiscard]] rmi::ReadUFrameDataPacket::Response
  readUFrameData(int number, std::optional<double> timeout = std::nullopt, std::optional<uint8_t> group = std::nullopt)
  {
    return rmi_connection_->readUFrameData(number, timeout, group);
  }

  rmi::WriteUFrameDataPacket::Response writeUFrameData(int number, rmi::FrameData data,
                                                       std::optional<double> timeout = std::nullopt,
                                                       std::optional<uint8_t> group = std::nullopt)
  {
    return rmi_connection_->writeUFrameData(number, data, timeout, group);
  }

  [[nodiscard]] rmi::ReadUToolDataPacket::Response
  readUToolData(int number, std::optional<double> timeout = std::nullopt, std::optional<uint8_t> group = std::nullopt)
  {
    return rmi_connection_->readUToolData(number, timeout, group);
  }

  rmi::WriteUToolDataPacket::Response writeUToolData(int number, rmi::FrameData data,
                                                     std::optional<double> timeout = std::nullopt,
                                                     std::optional<uint8_t> group = std::nullopt)
  {
    return rmi_connection_->writeUToolData(number, data, timeout, group);
  }

  [[nodiscard]] rmi::GetCartesianPositionPacket::Response
  getCartesianPosition(std::optional<double> timeout = std::nullopt, std::optional<uint8_t> group = std::nullopt)
  {
    return rmi_connection_->getCartesianPosition(timeout, group);
  }

  [[nodiscard]] rmi::GetTCPSpeedPacket::Response getTCPSpeed(std::optional<double> timeout = std::nullopt,
                                                             std::optional<uint8_t> group = std::nullopt)
  {
    return rmi_connection_->getTCPSpeed(timeout, group);
  }

  [[nodiscard]] rmi::GetPayloadPacket::Response getPayloadSchedule(std::optional<double> timeout = std::nullopt)
  {
    return rmi_connection_->getPayloadSchedule(timeout);
  }

  [[nodiscard]] rmi::GetPayloadValuePacket::Response getPayloadValue(uint16_t number,
                                                                     std::optional<double> timeout = std::nullopt)
  {
    return rmi_connection_->getPayloadValue(number, timeout);
  }

  [[nodiscard]] rmi::GetPayloadCompPacket::Response getPayloadComp(uint16_t number,
                                                                   std::optional<double> timeout = std::nullopt)
  {
    return rmi_connection_->getPayloadComp(number, timeout);
  }

  rmi::SetPayloadValuePacket::Response setPayloadValue(uint8_t number, float mass, float cg_x, float cg_y, float cg_z,
                                                       bool use_in, float in_x, float in_y, float in_z,
                                                       std::optional<double> timeout = std::nullopt)
  {
    return rmi_connection_->setPayloadValue(number, mass, cg_x, cg_y, cg_z, use_in, in_x, in_y, in_z, timeout);
  }

  rmi::SetPayloadCompPacket::Response setPayloadComp(uint8_t number, float mass, float cg_x, float cg_y, float cg_z,
                                                     float in_x, float in_y, float in_z,
                                                     std::optional<double> timeout = std::nullopt)
  {
    return rmi_connection_->setPayloadComp(number, mass, cg_x, cg_y, cg_z, in_x, in_y, in_z, timeout);
  }

  [[nodiscard]] std::optional<rmi::SystemFaultPacket> checkSystemFault()
  {
    return rmi_connection_->checkSystemFault();
  }

  [[nodiscard]] std::optional<rmi::TimeoutTerminatePacket> checkTimeoutTerminate()
  {
    return rmi_connection_->checkTimeoutTerminate();
  }

  [[nodiscard]] std::optional<rmi::CommunicationPacket> checkCommunicationPacket()
  {
    return rmi_connection_->checkCommunicationPacket();
  }

  [[nodiscard]] std::optional<rmi::UnknownPacket> checkUnknownPacket()
  {
    return rmi_connection_->checkUnknownPacket();
  }

  [[nodiscard]] std::optional<rmi::InstructionResponse> getLastInstructionResponse()
  {
    return rmi_connection_->getLastInstructionResponse();
  }

  [[nodiscard]] std::string getErrorMessageString(uint32_t code)
  {
    return rmi_connection_->getErrorMessageString(code);
  }

  [[nodiscard]] int32_t getRemainingBuffuerSize()
  {
    return rmi_connection_->getRemainingBuffuerSize();
  }

  void setEncoding(const std::string& encoding)
  {
    rmi_connection_->setEncoding(encoding);
  }

  bool startMotionControl();

  void stopMotionControl();

  bool getDoMotnCtrl() const
  {
    return do_motn_ctrl_;
  }

  void setDoMotnCtrl(const bool do_motn_ctrl)
  {
    do_motn_ctrl_ = do_motn_ctrl;
  }

  bool getLimits(double v_peak, double payload, std::vector<double>& vel_limit, std::vector<double>& acc_limit,
                 std::vector<double>& jerk_limit) const;

  uint32_t getControlPeriod() const;

  void setPayloadSchedule(uint8_t payload_schedule) const;

  void validateGPIOBuffer(const std::shared_ptr<GPIOBuffer>& gpio_buffer) const;

  void setOutCmdInterpBuffTarget(uint32_t out_cmd_interp_buff_target)
  {
    out_cmd_interp_buff_target_ = out_cmd_interp_buff_target;
  }

  uint32_t getOutCmdInterpBuffTarget() const
  {
    return out_cmd_interp_buff_target_;
  }

  void setForceSensorType(uint32_t force_sensor_type)
  {
    force_sensor_type_ = force_sensor_type;
  }

  uint32_t getForceSensorType() const
  {
    return force_sensor_type_;
  }

  [[nodiscard]] const RobotStatus& robot_status() const
  {
    return robot_status_;
  }

  [[nodiscard]] const ForceSensor& force_sensor() const
  {
    return force_sensor_;
  }

  [[nodiscard]] RobotStatus readRobotStatus()
  {
    readStateFromQueue();
    return robot_status_;
  }

  [[nodiscard]] ForceSensor readForceSensor()
  {
    readStateFromQueue();
    return force_sensor_;
  }

  void configureForceSensor(uint32_t do_reset, uint32_t force_sensor_type) const;

  // Get client (stream motion) version
  uint32_t getClientVersion() const
  {
    return client_version_;
  }

private:
  void assertRMIMotionAllowed() const;

  GPIOBuffer& gpio_buffer();

  void readStateFromQueue();

  void streamMotionThread(std::stop_token stop_token, const Eigen::VectorXd& joint_angles);

  /** Grab the limits from the robot.*/
  void fetchRobotLimits();

  const std::string robot_ip_;
  const uint16_t stream_motion_port_;
  const uint16_t rmi_port_;

  // Limits
  Eigen::MatrixXd vel_limits_no_load_ = Eigen::MatrixXd::Zero(9, 20);
  Eigen::MatrixXd acc_limits_no_load_ = Eigen::MatrixXd::Zero(9, 20);
  Eigen::MatrixXd jerk_limits_no_load_ = Eigen::MatrixXd::Zero(9, 20);
  Eigen::MatrixXd vel_limits_full_load_ = Eigen::MatrixXd::Zero(9, 20);
  Eigen::MatrixXd acc_limits_full_load_ = Eigen::MatrixXd::Zero(9, 20);
  Eigen::MatrixXd jerk_limits_full_load_ = Eigen::MatrixXd::Zero(9, 20);

  // Manages stream motion connection
  std::atomic<bool> is_streaming_ = false;
  std::chrono::time_point<std::chrono::steady_clock> start_time_;
  std::unique_ptr<stream_motion::StreamMotionInterface> stream_motion_;

  std::array<double, stream_motion::kMaxAxisNumber> command_pos;
  Eigen::VectorXd last_joint_angles_ = Eigen::VectorXd::Zero(9);
  Eigen::VectorXd last_joint_angles_cmd_ = Eigen::VectorXd::Zero(9);
  RobotStatus robot_status_{};
  ForceSensor force_sensor_{};
  bool in_motion_ = false;
  uint32_t control_period_ = 0;
  uint32_t client_version_ = 0;  // stream motion client version

  // IO data only accessed from the non-realitime thread.
  std::shared_ptr<GPIOBuffer> gpio_buffer_;

  // Real time thread data
  std::jthread rt_thread_;

  std::atomic<bool> do_motn_ctrl_ = true;

  // Manages RMI connection
  std::unique_ptr<rmi::RMIConnectionInterface> rmi_connection_;
  std::atomic<bool> rmi_running_ = false;

  // Output command interpolation buffer target size for stream motion control
  std::atomic<uint32_t> out_cmd_interp_buff_target_;

  // Force sensor default type
  uint32_t force_sensor_type_;

  struct PQueueImpl;
  std::unique_ptr<PQueueImpl> p_queue_impl_;
};
