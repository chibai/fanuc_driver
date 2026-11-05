// SPDX-FileCopyrightText: 2025-2026, FANUC America Corporation
// SPDX-FileCopyrightText: 2025-2026, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>
#include <list>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "fanuc_driver/packets.hpp"
#include "fanuc_driver/types.hpp"
#include "robot_driver/robot_driver.hpp"

class Fanuc_driver final : public Robot_driver
{
public:
  explicit Fanuc_driver(const std::string& robot_ip_address, uint16_t rmi_port = 16001);

  ~Fanuc_driver() override;

  Fanuc_driver(const Fanuc_driver&) = delete;

  Fanuc_driver& operator=(const Fanuc_driver&) = delete;

  rmi::ConnectPacket::Response connect(std::optional<double> timeout = std::nullopt);

  rmi::DisconnectPacket::Response disconnect(std::optional<double> timeout = std::nullopt);

  rmi::InitializePacket::Response initializeRemoteMotion(std::optional<double> timeout = std::nullopt);
  rmi::InitializePacket::Response initializeRemoteMotion(std::optional<double> timeout,
                                                         const std::optional<uint8_t> groupmask,
                                                         const std::optional<std::string>& rtsa,
                                                         const std::optional<std::string>& pltzmode);

  rmi::ProgramCallPacket::Response programCall(const std::string& program_name,
                                               std::optional<double> timeout = std::nullopt);

  rmi::ProgramCallPacket::Response programCall(const std::string& program_name, std::optional<double> timeout,
                                               const std::vector<rmi::RMICallParam>& params);

  rmi::ProgramCallPacket::Request programCallNonBlocking(const std::string& program_name);

  rmi::ProgramCallPacket::Request programCallNonBlocking(const std::string& program_name,
                                                         const std::vector<rmi::RMICallParam>& params);

  rmi::StatusRequestPacket::Response getStatus(std::optional<double> timeout = std::nullopt);

  rmi::SetSpeedOverridePacket::Response setSpeedOverride(int value, std::optional<double> timeout = std::nullopt);

  rmi::AbortPacket::Response abort(std::optional<double> timeout = std::nullopt);

  rmi::PausePacket::Response pause(std::optional<double> timeout = std::nullopt);

  rmi::ContinuePacket::Response resume(std::optional<double> timeout = std::nullopt);

  rmi::ResetRobotPacket::Response reset(std::optional<double> timeout = std::nullopt);

  rmi::ReadErrorPacket::Response readError(std::optional<double> timeout = std::nullopt);

  rmi::ReadErrorPacket::Response readError(std::optional<double> timeout, const std::optional<uint8_t> count);

  rmi::GetUFrameToolFramePacket::Response getUFrameUTool(std::optional<double> timeout = std::nullopt,
                                                         const std::optional<uint8_t> group = std::nullopt);

  rmi::SetUFrameToolFramePacket::Response setUFrameUTool(const int uframe, const int utool,
                                                         std::optional<double> timeout = std::nullopt,
                                                         const std::optional<uint8_t> group = std::nullopt);

  rmi::ReadUFrameDataPacket::Response readUFrameData(const int uframe, std::optional<double> timeout = std::nullopt,
                                                     const std::optional<uint8_t> group = std::nullopt);

  rmi::WriteUFrameDataPacket::Response writeUFrameData(const int uframe, const rmi::FrameData data,
                                                       std::optional<double> timeout = std::nullopt,
                                                       const std::optional<uint8_t> group = std::nullopt);

  rmi::ReadUToolDataPacket::Response readUToolData(const int utool, std::optional<double> timeout = std::nullopt,
                                                   const std::optional<uint8_t> group = std::nullopt);

  rmi::WriteUToolDataPacket::Response writeUToolData(const int utool, const rmi::FrameData data,
                                                     std::optional<double> timeout = std::nullopt,
                                                     const std::optional<uint8_t> group = std::nullopt);

  rmi::WritePositionRegisterPacket::Response
  writePositionRegister(int register_number, const std::string& representation,
                        const rmi::ConfigurationData& configuration, const rmi::PositionData& position,
                        const rmi::JointAngleData& joint_angle, std::optional<double> timeout = std::nullopt);

  rmi::ReadPositionRegisterPacket::Response readPositionRegister(int register_number,
                                                                 std::optional<double> timeout = std::nullopt);

  rmi::ReadNumericRegisterPacket::Response readNumericRegister(int register_number,
                                                               std::optional<double> timeout = std::nullopt);

  rmi::WriteNumericRegisterPacket::Response writeNumericRegister(int register_number, std::variant<int, float> value,
                                                                 std::optional<double> timeout = std::nullopt);

  rmi::ReadDigitalInputPortPacket::Response readDigitalInputPort(uint16_t port_number,
                                                                 std::optional<double> timeout = std::nullopt);

  rmi::WriteDigitalOutputPacket::Response writeDigitalOutputPort(uint16_t port_number, bool port_value,
                                                                 std::optional<double> timeout = std::nullopt);

  rmi::ReadIOPortPacket::Response readIOPort(const std::string& port_type, int port_number,
                                             std::optional<double> timeout = std::nullopt);

  rmi::WriteIOPortPacket::Response writeIOPort(int port_number, const std::string& port_type,
                                               std::variant<int, float> port_value,
                                               std::optional<double> timeout = std::nullopt);

  rmi::ReadVariablePacket::Response readVariablePacket(const std::string& variable_name,
                                                       std::optional<double> timeout = std::nullopt);

  rmi::WriteVariablePacket::Response writeVariablePacket(const std::string& variable_name,
                                                         std::variant<int, float> value,
                                                         std::optional<double> timeout = std::nullopt);

  rmi::GetExtendedStatusPacket::Response getExtendedStatus(std::optional<double> timeout = std::nullopt);

  rmi::SetPayloadPacket::Response setPayloadSchedule(uint8_t payload_schedule_number,
                                                     std::optional<double> timeout = std::nullopt);

  rmi::SetPayloadValuePacket::Response setPayloadValue(uint8_t payload_schedule_number, float mass, float cg_x,
                                                       float cg_y, float cg_z, bool use_in, float in_x, float in_y,
                                                       float in_z, std::optional<double> timeout = std::nullopt);

  rmi::SetPayloadCompPacket::Response setPayloadComp(uint8_t payload_schedule_number, float mass, float cg_x,
                                                     float cg_y, float cg_z, float in_x, float in_y, float in_z,
                                                     std::optional<double> timeout = std::nullopt);

  rmi::GetPayloadPacket::Response getPayloadSchedule(std::optional<double> timeout = std::nullopt);

  rmi::GetPayloadValuePacket::Response getPayloadValue(uint16_t payload_schedule_number,
                                                       std::optional<double> timeout = std::nullopt);

  rmi::GetPayloadCompPacket::Response getPayloadComp(uint16_t payload_schedule_number,
                                                     std::optional<double> timeout = std::nullopt);

  rmi::ReadJointAnglesPacket::Response readJointAngles(const std::optional<uint8_t>& group = std::nullopt,
                                                       std::optional<double> timeout = std::nullopt);
  rmi::GetCartesianPositionPacket::Response getCartesianPosition(std::optional<double> timeout = std::nullopt,
                                                                 const std::optional<uint8_t> group = std::nullopt);
  rmi::GetTCPSpeedPacket::Response getTCPSpeed(std::optional<double> timeout = std::nullopt,
                                               const std::optional<uint8_t> group = std::nullopt);

  rmi::JointMotionJRepPacket::Response sendJointMotion(rmi::JointMotionJRepPacket::Request joint_motion_request,
                                                       std::optional<double> timeout = std::nullopt);

  void sendRMIPacketNonBlocking(rmi::JointMotionPacket::Request& packet);
  void sendRMIPacketNonBlocking(rmi::JointMotionJRepPacket::Request& packet);
  void sendRMIPacketNonBlocking(rmi::JointRelativePacket::Request& packet);
  void sendRMIPacketNonBlocking(rmi::JointRelativeJRepPacket::Request& packet);

  void sendRMIPacketNonBlocking(rmi::LinearMotionPacket::Request& packet);
  void sendRMIPacketNonBlocking(rmi::LinearMotionJRepPacket::Request& packet);
  void sendRMIPacketNonBlocking(rmi::LinearRelativePacket::Request& packet);
  void sendRMIPacketNonBlocking(rmi::LinearRelativeJRepPacket::Request& packet);

  void sendRMIPacketNonBlocking(rmi::CircularMotionPacket::Request& packet);
  void sendRMIPacketNonBlocking(rmi::CircularRelativePacket::Request& packet);

  void sendRMIPacketNonBlocking(rmi::SplineMotionPacket::Request& packet);
  void sendRMIPacketNonBlocking(rmi::SplineMotionJRepPacket::Request& packet);

  void sendRMIPacketNonBlocking(rmi::WaitForDINPacket::Request& packet);
  void sendRMIPacketNonBlocking(rmi::SetUFramePacket::Request& packet);
  void sendRMIPacketNonBlocking(rmi::SetToolFramePacket::Request& packet);
  void sendRMIPacketNonBlocking(rmi::WaitForTimePacket::Request& packet);
  void sendRMIPacketNonBlocking(rmi::SetPayloadInstructionPacket::Request& packet);

  std::string getErrorMessageString(const uint32_t error_code);

  std::optional<rmi::SystemFaultPacket> checkSystemFault();

  std::optional<rmi::TimeoutTerminatePacket> checkTimeoutTerminate();

  std::optional<rmi::CommunicationPacket> checkCommunicationPacket();

  std::optional<rmi::UnknownPacket> checkUnknownPacket();

  std::optional<rmi::InstructionResponse> getLastInstructionResponse();

  int32_t getRemainingBufferSize();

  void setEncoding(const std::string& encoding);

  template <typename T>
  typename T::Response sendRMIPacket(typename T::Request& request_packet, std::optional<double> timeout = std::nullopt);

private:
  void doConnect(std::optional<double> timeout) override;
  void doDisconnect(std::optional<double> timeout) override;
  void doInitializeRemoteMotion(std::optional<double> timeout) override;
  void doPause(std::optional<double> timeout) override;
  void doResume(std::optional<double> timeout) override;
  void doAbort(std::optional<double> timeout) override;
  void doReset(std::optional<double> timeout) override;
  void doSetSpeedOverride(int value, std::optional<double> timeout) override;
  RobotStatus doGetStatus(std::optional<double> timeout) override;
  bool doReadDigitalInputPort(uint16_t port_number, std::optional<double> timeout) override;
  void doWriteDigitalOutputPort(uint16_t port_number, bool port_value, std::optional<double> timeout) override;

  struct PConnectionImpl;

  int32_t getSequenceNumber();

  template <typename T>
  T getResponsePacket(std::optional<double> timeout_optional, const std::string& error_message_prefix,
                      std::optional<int> expected_sequence_id);

  template <typename T>
  std::optional<T> checkPushPacket();

  void drainConnectionBuffer();
  bool processInstructionResponse(std::string& json_response);
  template <typename T>
  std::optional<T> checkForPacketInJSONResponses(std::list<std::string>::iterator& it,
                                                 std::list<std::string>& json_responses,
                                                 std::optional<int> expected_sequence_id = std::nullopt);

  template <typename T>
  void sendRMIPacketNonBlockingImpl(typename T::Request& request_packet);

  const std::string robot_ip_address_;
  const uint16_t rmi_port_;

  std::string encoding_;
  int32_t sequence_number_;
  std::list<std::string> json_responses_;
  mutable std::mutex mutex_;
  mutable std::mutex motion_mutex_;

  const std::unique_ptr<PConnectionImpl> connection_impl_;

  std::optional<rmi::InstructionResponse> last_instruction_response_;
  mutable std::mutex instruction_mutex_;
};
