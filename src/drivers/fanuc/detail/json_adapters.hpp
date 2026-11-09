// SPDX-FileCopyrightText: 2026, FANUC America Corporation
// SPDX-FileCopyrightText: 2026, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <optional>
#include <string>
#include <variant>

#include <nlohmann/json.hpp>

#include "packets.hpp"
#include "types.hpp"

namespace nlohmann
{
template <typename T>
struct adl_serializer<std::optional<T>>
{
  static void to_json(json& value, const std::optional<T>& data)
  {
    if (data)
    {
      value = *data;
    }
    else
    {
      value = nullptr;
    }
  }

  static void from_json(const json& value, std::optional<T>& data)
  {
    if (value.is_null())
    {
      data.reset();
    }
    else
    {
      data = value.get<T>();
    }
  }
};

template <>
struct adl_serializer<std::variant<int, float, std::string>>
{
  static void to_json(json& value, const std::variant<int, float, std::string>& data)
  {
    std::visit([&value](const auto& item) { value = item; }, data);
  }

  static void from_json(const json& value, std::variant<int, float, std::string>& data)
  {
    if (value.is_string())
    {
      data = value.get<std::string>();
    }
    else if (value.is_number_integer())
    {
      data = value.get<int>();
    }
    else
    {
      data = value.get<float>();
    }
  }
};

template <>
struct adl_serializer<std::variant<int, float>>
{
  static void to_json(json& value, const std::variant<int, float>& data)
  {
    std::visit([&value](const auto& item) { value = item; }, data);
  }

  static void from_json(const json& value, std::variant<int, float>& data)
  {
    if (value.is_number_integer())
    {
      data = value.get<int>();
    }
    else
    {
      data = value.get<float>();
    }
  }
};
}  // namespace nlohmann

namespace rmi
{
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(InstructionResponse, Instruction, ErrorID, SequenceID)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(FrameData, X, Y, Z, W, P, R)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(PositionData, X, Y, Z, W, P, R, Ext1, Ext2, Ext3)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ConfigurationData, UToolNumber, UFrameNumber, Front, Up, Left, Flip,
                                                Turn4, Turn5, Turn6)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(JointAngleData, J1, J2, J3, J4, J5, J6, J7, J8, J9)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(CartesianMotionGroup, Configuration, Position)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ConnectPacket::Request, Communication)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ConnectPacket::Response, Communication, ErrorID, PortNumber,
                                                MajorVersion, MinorVersion)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(DisconnectPacket::Request, Communication)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(DisconnectPacket::Response, Communication, ErrorID)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(TimeoutTerminatePacket, Communication)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(SystemFaultPacket, Communication, SequenceID)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(InitializePacket::Request, Command, GroupMask, RTSA, PLTZMODE)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(InitializePacket::Response, Command, ErrorID, GroupMask)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(AbortPacket::Request, Command)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(AbortPacket::Response, Command, ErrorID)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(PausePacket::Request, Command)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(PausePacket::Response, Command, ErrorID)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ContinuePacket::Request, Command)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ContinuePacket::Response, Command, ErrorID)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ReadErrorPacket::Request, Command, Count)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ReadErrorPacket::Response, Command, ErrorID, Count, ErrorData,
                                                ErrorData2, ErrorData3, ErrorData4, ErrorData5)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(SetUFrameToolFramePacket::Request, Command, UFrameNumber, UToolNumber,
                                                Group)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(SetUFrameToolFramePacket::Response, Command, ErrorID, Group)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(StatusRequestPacket::Request, Command)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(StatusRequestPacket::Response, Command, ErrorID, ServoReady, TPMode,
                                                RMIMotionStatus, ProgramStatus, SingleStepMode, NumberUTool,
                                                NextSequenceID, NumberUFrame, Override)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ReadUFrameDataPacket::Request, Command, FrameNumber, Group)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ReadUFrameDataPacket::Response, Command, ErrorID, FrameNumber, Frame,
                                                Group)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(WriteUFrameDataPacket::Request, Command, FrameNumber, Frame, Group)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(WriteUFrameDataPacket::Response, Command, ErrorID, Group)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ReadUToolDataPacket::Request, Command, ToolNumber, Group)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ReadUToolDataPacket::Response, Command, ErrorID, ToolNumber, Frame,
                                                Group)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(WriteUToolDataPacket::Request, Command, ToolNumber, Frame, Group)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(WriteUToolDataPacket::Response, Command, ErrorID, Group)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ReadDigitalInputPortPacket::Request, Command, PortNumber)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ReadDigitalInputPortPacket::Response, Command, ErrorID, PortNumber,
                                                PortValue)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(WriteDigitalOutputPacket::Request, Command, PortNumber, PortValue)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(WriteDigitalOutputPacket::Response, Command, ErrorID)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(GetCartesianPositionPacket::Request, Command, Group)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(GetCartesianPositionPacket::Response, Command, ErrorID, TimeTag,
                                                Configuration, Position, Group)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ReadJointAnglesPacket::Request, Command, Group)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ReadJointAnglesPacket::Response, Command, ErrorID, TimeTag, JointAngle,
                                                Group)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(SetSpeedOverridePacket::Request, Command, Value)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(SetSpeedOverridePacket::Response, Command, ErrorID)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(GetUFrameToolFramePacket::Request, Command, Group)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(GetUFrameToolFramePacket::Response, Command, ErrorID, UFrameNumber,
                                                UToolNumber, Group)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ReadPositionRegisterPacket::Request, Command, RegisterNumber, Group)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ReadPositionRegisterPacket::Response, Command, ErrorID, RegisterNumber,
                                                Representation, Configuration, Position, JointAngle, Group)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(WritePositionRegisterPacket::Request, Command, RegisterNumber,
                                                Representation, Configuration, Position, JointAngle, Group)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(WritePositionRegisterPacket::Response, Command, ErrorID, RegisterNumber,
                                                Group)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ResetRobotPacket::Request, Command)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ResetRobotPacket::Response, Command, ErrorID)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(GetTCPSpeedPacket::Request, Command, Group)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(GetTCPSpeedPacket::Response, Command, ErrorID, TimeTag, Speed, Group)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(WaitForDINPacket::Request, Instruction, SequenceID, PortNumber,
                                                PortValue)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(WaitForDINPacket::Response, Instruction, ErrorID, SequenceID)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(SetUFramePacket::Request, Instruction, SequenceID, FrameNumber)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(SetUFramePacket::Response, Instruction, ErrorID, SequenceID)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(SetToolFramePacket::Request, Instruction, SequenceID, ToolNumber)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(SetToolFramePacket::Response, Instruction, ErrorID, SequenceID)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(WaitForTimePacket::Request, Instruction, SequenceID, Time)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(WaitForTimePacket::Response, Instruction, ErrorID, SequenceID)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(SetPayloadInstructionPacket::Request, Instruction, SequenceID,
                                                ScheduleNumber)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(SetPayloadInstructionPacket::Response, Instruction, ErrorID, SequenceID)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(SetPayloadPacket::Request, Command, ScheduleNumber, Group)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(SetPayloadPacket::Response, Command, ErrorID)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(GetPayloadPacket::Request, Command, Group)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(GetPayloadPacket::Response, Command, ScheduleNumber, ErrorID)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ProgramCallPacket::Request, Instruction, SequenceID, ProgramName,
                                                ParamType1, ParamValue1, ParamType2, ParamValue2, ParamType3,
                                                ParamValue3, ParamType4, ParamValue4, ParamType5, ParamValue5,
                                                ParamType6, ParamValue6, ParamType7, ParamValue7, ParamType8,
                                                ParamValue8, ParamType9, ParamValue9, ParamType10, ParamValue10)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ProgramCallPacket::Response, Instruction, ErrorID, SequenceID)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(LinearMotionPacket::Request, Instruction, SequenceID, Configuration,
                                                Position, SpeedType, Speed, TermType, TermValue, ACC, OffsetPRNumber,
                                                VisionPRNumber, WristJoint, MROT, LCBType, LCBValue, PortType,
                                                portNumber, portValue, ToolOffsetPRNumber, ALIM, ALIMREG, NoBlend)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(LinearMotionPacket::Response, Instruction, ErrorID, SequenceID)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(LinearRelativePacket::Request, Instruction, SequenceID, Configuration,
                                                Position, SpeedType, Speed, TermType, TermValue, ACC, OffsetPRNumber,
                                                VisionPRNumber, WristJoint, MROT, LCBType, LCBValue, PortType,
                                                portNumber, portValue, ToolOffsetPRNumber, ALIM, ALIMREG, NoBlend)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(LinearRelativePacket::Response, Instruction, ErrorID, SequenceID)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(JointMotionPacket::Request, Instruction, SequenceID, Configuration,
                                                Position, SpeedType, Speed, TermType, TermValue, ACC, OffsetPRNumber,
                                                VisionPRNumber, MROT, LCBType, LCBValue, PortType, portNumber,
                                                portValue, ToolOffsetPRNumber, NoBlend)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(JointMotionPacket::Response, Instruction, ErrorID, SequenceID)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(JointRelativePacket::Request, Instruction, SequenceID, Configuration,
                                                Position, SpeedType, Speed, TermType, TermValue, ACC, OffsetPRNumber,
                                                VisionPRNumber, MROT, LCBType, LCBValue, PortType, portNumber,
                                                portValue, ToolOffsetPRNumber, NoBlend)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(JointRelativePacket::Response, Instruction, ErrorID, SequenceID)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(CircularMotionPacket::Request, Instruction, SequenceID, Configuration,
                                                Position, ViaConfiguration, ViaPosition, SpeedType, Speed, TermType,
                                                TermValue, ACC, OffsetPRNumber, VisionPRNumber, WristJoint, MROT,
                                                LCBType, LCBValue, PortType, portNumber, portValue, ToolOffsetPRNumber,
                                                NoBlend)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(CircularMotionPacket::Response, Instruction, ErrorID, SequenceID)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(CircularRelativePacket::Request, Instruction, SequenceID, Configuration,
                                                Position, ViaConfiguration, ViaPosition, SpeedType, Speed, TermType,
                                                TermValue, ACC, OffsetPRNumber, VisionPRNumber, WristJoint, MROT,
                                                LCBType, LCBValue, PortType, portNumber, portValue, ToolOffsetPRNumber,
                                                NoBlend)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(CircularRelativePacket::Response, Instruction, ErrorID, SequenceID)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(JointMotionJRepPacket::Request, Instruction, SequenceID, JointAngle,
                                                SpeedType, Speed, TermType, TermValue, ACC, OffsetPRNumber,
                                                VisionPRNumber, MROT, LCBType, LCBValue, PortType, portNumber,
                                                portValue, ToolOffsetPRNumber, NoBlend)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(JointMotionJRepPacket::Response, Instruction, ErrorID, SequenceID)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(JointRelativeJRepPacket::Request, Instruction, SequenceID, JointAngle,
                                                SpeedType, Speed, TermType, TermValue, ACC, OffsetPRNumber,
                                                VisionPRNumber, MROT, LCBType, LCBValue, PortType, portNumber,
                                                portValue, ToolOffsetPRNumber, NoBlend)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(JointRelativeJRepPacket::Response, Instruction, ErrorID, SequenceID)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(LinearMotionJRepPacket::Request, Instruction, SequenceID, JointAngle,
                                                SpeedType, Speed, TermType, TermValue, ACC, OffsetPRNumber,
                                                VisionPRNumber, WristJoint, MROT, LCBType, LCBValue, PortType,
                                                portNumber, portValue, ToolOffsetPRNumber, ALIM, ALIMREG, NoBlend)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(LinearMotionJRepPacket::Response, Instruction, ErrorID, SequenceID)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(LinearRelativeJRepPacket::Request, Instruction, SequenceID, JointAngle,
                                                SpeedType, Speed, TermType, TermValue, ACC, OffsetPRNumber,
                                                VisionPRNumber, WristJoint, MROT, LCBType, LCBValue, PortType,
                                                portNumber, portValue, ToolOffsetPRNumber, ALIM, ALIMREG, NoBlend)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(LinearRelativeJRepPacket::Response, Instruction, ErrorID, SequenceID)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(SplineMotionPacket::Request, Instruction, SequenceID, Configuration,
                                                Position, SpeedType, Speed, TermType, TermValue, ACC, OffsetPRNumber,
                                                VisionPRNumber, LCBType, LCBValue, PortType, portNumber, portValue,
                                                ToolOffsetPRNumber)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(SplineMotionPacket::Response, Instruction, ErrorID, SequenceID)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(SplineMotionJRepPacket::Request, Instruction, SequenceID, JointAngle,
                                                SpeedType, Speed, TermType, TermValue, ACC, OffsetPRNumber,
                                                VisionPRNumber, LCBType, LCBValue, PortType, portNumber, portValue,
                                                ToolOffsetPRNumber)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(SplineMotionJRepPacket::Response, Instruction, ErrorID, SequenceID)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(UnknownPacket, Command, ErrorID, SequenceID)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(CommunicationPacket, Communication)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ReadNumericRegisterPacket::Request, Command, RegisterNumber)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ReadNumericRegisterPacket::Response, Command, ErrorID, DataType,
                                                RegisterValue)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(WriteNumericRegisterPacket::Request, Command, RegisterNumber,
                                                RegisterValue, DataType)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(WriteNumericRegisterPacket::Response, Command, ErrorID)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ReadIOPortPacket::Request, Command, PortType, PortNumber)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ReadIOPortPacket::Response, Command, ErrorID, PortType, PortNumber,
                                                PortValue)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(WriteIOPortPacket::Request, Command, PortType, PortNumber, PortValue)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(WriteIOPortPacket::Response, Command, ErrorID, PortType, PortNumber)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ReadVariablePacket::Request, Command, VariableName)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(ReadVariablePacket::Response, Command, ErrorID, VariableName,
                                                VariableType, VariableValue)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(WriteVariablePacket::Request, Command, VariableName, VariableType,
                                                VariableValue)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(WriteVariablePacket::Response, Command, ErrorID, VariableName)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(GetExtendedStatusPacket::Request, Command)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(GetExtendedStatusPacket::Response, Command, ErrorID, ErrorCode,
                                                InMotion, ControlMode, DrivesPowered, GenOverride, SpeedClampLimit)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(SetPayloadValuePacket::Request, Command, ScheduleNumber, Group, Mass,
                                                CG_X, CG_Y, CG_Z, IN_X, IN_Y, IN_Z)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(SetPayloadValuePacket::Response, Command, ErrorID)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(GetPayloadValuePacket::Request, Command, Group)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(GetPayloadValuePacket::Response, Command, Mass, CG_X, CG_Y, CG_Z, IN_X,
                                                IN_Y, IN_Z, ErrorID)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(SetPayloadCompPacket::Request, Command, ScheduleNumber, Group, Mass,
                                                CG_X, CG_Y, CG_Z, IN_X, IN_Y, IN_Z)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(SetPayloadCompPacket::Response, Command, ErrorID)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(GetPayloadCompPacket::Request, Command, ScheduleNumber, Group)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(GetPayloadCompPacket::Response, Command, Mass, CG_X, CG_Y, CG_Z, IN_X,
                                                IN_Y, IN_Z, ErrorID)
}  // namespace rmi

namespace rmi::json_detail
{
template <typename T>
bool hasRequiredFields(const nlohmann::json& document);

template <>
inline bool hasRequiredFields<InstructionResponse>(const nlohmann::json& document)
{
  return document.contains("Instruction") && document.contains("ErrorID") && document.contains("SequenceID");
}
template <>
inline bool hasRequiredFields<FrameData>(const nlohmann::json& document)
{
  return document.contains("X") && document.contains("Y") && document.contains("Z") && document.contains("W") &&
         document.contains("P") && document.contains("R");
}
template <>
inline bool hasRequiredFields<PositionData>(const nlohmann::json& document)
{
  return document.contains("X") && document.contains("Y") && document.contains("Z") && document.contains("W") &&
         document.contains("P") && document.contains("R") && document.contains("Ext1") && document.contains("Ext2") &&
         document.contains("Ext3");
}
template <>
inline bool hasRequiredFields<ConfigurationData>(const nlohmann::json& document)
{
  return document.contains("UToolNumber") && document.contains("UFrameNumber") && document.contains("Front") &&
         document.contains("Up") && document.contains("Left") && document.contains("Flip") &&
         document.contains("Turn4") && document.contains("Turn5") && document.contains("Turn6");
}
template <>
inline bool hasRequiredFields<JointAngleData>(const nlohmann::json& document)
{
  return document.contains("J1") && document.contains("J2") && document.contains("J3") && document.contains("J4") &&
         document.contains("J5") && document.contains("J6") && document.contains("J7") && document.contains("J8") &&
         document.contains("J9");
}
template <>
inline bool hasRequiredFields<CartesianMotionGroup>(const nlohmann::json& document)
{
  return document.contains("Configuration") && document.contains("Position");
}
template <>
inline bool hasRequiredFields<ConnectPacket::Request>(const nlohmann::json& document)
{
  return document.contains("Communication");
}
template <>
inline bool hasRequiredFields<ConnectPacket::Response>(const nlohmann::json& document)
{
  return document.contains("Communication") && document.contains("ErrorID") && document.contains("PortNumber") &&
         document.contains("MajorVersion") && document.contains("MinorVersion");
}
template <>
inline bool hasRequiredFields<DisconnectPacket::Request>(const nlohmann::json& document)
{
  return document.contains("Communication");
}
template <>
inline bool hasRequiredFields<DisconnectPacket::Response>(const nlohmann::json& document)
{
  return document.contains("Communication") && document.contains("ErrorID");
}
template <>
inline bool hasRequiredFields<TimeoutTerminatePacket>(const nlohmann::json& document)
{
  return document.contains("Communication");
}
template <>
inline bool hasRequiredFields<SystemFaultPacket>(const nlohmann::json& document)
{
  return document.contains("Communication") && document.contains("SequenceID");
}
template <>
inline bool hasRequiredFields<InitializePacket::Request>(const nlohmann::json& document)
{
  return document.contains("Command");
}
template <>
inline bool hasRequiredFields<InitializePacket::Response>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("ErrorID");
}
template <>
inline bool hasRequiredFields<AbortPacket::Request>(const nlohmann::json& document)
{
  return document.contains("Command");
}
template <>
inline bool hasRequiredFields<AbortPacket::Response>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("ErrorID");
}
template <>
inline bool hasRequiredFields<PausePacket::Request>(const nlohmann::json& document)
{
  return document.contains("Command");
}
template <>
inline bool hasRequiredFields<PausePacket::Response>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("ErrorID");
}
template <>
inline bool hasRequiredFields<ContinuePacket::Request>(const nlohmann::json& document)
{
  return document.contains("Command");
}
template <>
inline bool hasRequiredFields<ContinuePacket::Response>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("ErrorID");
}
template <>
inline bool hasRequiredFields<ReadErrorPacket::Request>(const nlohmann::json& document)
{
  return document.contains("Command");
}
template <>
inline bool hasRequiredFields<ReadErrorPacket::Response>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("ErrorID") && document.contains("ErrorData");
}
template <>
inline bool hasRequiredFields<SetUFrameToolFramePacket::Request>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("UFrameNumber") && document.contains("UToolNumber");
}
template <>
inline bool hasRequiredFields<SetUFrameToolFramePacket::Response>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("ErrorID");
}
template <>
inline bool hasRequiredFields<StatusRequestPacket::Request>(const nlohmann::json& document)
{
  return document.contains("Command");
}
template <>
inline bool hasRequiredFields<StatusRequestPacket::Response>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("ErrorID") && document.contains("ServoReady") &&
         document.contains("TPMode") && document.contains("RMIMotionStatus") && document.contains("ProgramStatus") &&
         document.contains("SingleStepMode") && document.contains("NumberUTool") &&
         document.contains("NextSequenceID") && document.contains("NumberUFrame") && document.contains("Override");
}
template <>
inline bool hasRequiredFields<ReadUFrameDataPacket::Request>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("FrameNumber");
}
template <>
inline bool hasRequiredFields<ReadUFrameDataPacket::Response>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("ErrorID") && document.contains("FrameNumber") &&
         document.contains("Frame");
}
template <>
inline bool hasRequiredFields<WriteUFrameDataPacket::Request>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("FrameNumber") && document.contains("Frame");
}
template <>
inline bool hasRequiredFields<WriteUFrameDataPacket::Response>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("ErrorID");
}
template <>
inline bool hasRequiredFields<ReadUToolDataPacket::Request>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("ToolNumber");
}
template <>
inline bool hasRequiredFields<ReadUToolDataPacket::Response>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("ErrorID") && document.contains("ToolNumber") &&
         document.contains("Frame");
}
template <>
inline bool hasRequiredFields<WriteUToolDataPacket::Request>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("ToolNumber") && document.contains("Frame");
}
template <>
inline bool hasRequiredFields<WriteUToolDataPacket::Response>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("ErrorID");
}
template <>
inline bool hasRequiredFields<ReadDigitalInputPortPacket::Request>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("PortNumber");
}
template <>
inline bool hasRequiredFields<ReadDigitalInputPortPacket::Response>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("ErrorID") && document.contains("PortNumber") &&
         document.contains("PortValue");
}
template <>
inline bool hasRequiredFields<WriteDigitalOutputPacket::Request>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("PortNumber") && document.contains("PortValue");
}
template <>
inline bool hasRequiredFields<WriteDigitalOutputPacket::Response>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("ErrorID");
}
template <>
inline bool hasRequiredFields<GetCartesianPositionPacket::Request>(const nlohmann::json& document)
{
  return document.contains("Command");
}
template <>
inline bool hasRequiredFields<GetCartesianPositionPacket::Response>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("ErrorID") && document.contains("TimeTag") &&
         document.contains("Configuration") && document.contains("Position");
}
template <>
inline bool hasRequiredFields<ReadJointAnglesPacket::Request>(const nlohmann::json& document)
{
  return document.contains("Command");
}
template <>
inline bool hasRequiredFields<ReadJointAnglesPacket::Response>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("ErrorID") && document.contains("TimeTag") &&
         document.contains("JointAngle");
}
template <>
inline bool hasRequiredFields<SetSpeedOverridePacket::Request>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("Value");
}
template <>
inline bool hasRequiredFields<SetSpeedOverridePacket::Response>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("ErrorID");
}
template <>
inline bool hasRequiredFields<GetUFrameToolFramePacket::Request>(const nlohmann::json& document)
{
  return document.contains("Command");
}
template <>
inline bool hasRequiredFields<GetUFrameToolFramePacket::Response>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("ErrorID") && document.contains("UFrameNumber") &&
         document.contains("UToolNumber");
}
template <>
inline bool hasRequiredFields<ReadPositionRegisterPacket::Request>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("RegisterNumber");
}
template <>
inline bool hasRequiredFields<ReadPositionRegisterPacket::Response>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("ErrorID") && document.contains("RegisterNumber") &&
         document.contains("Representation");
}
template <>
inline bool hasRequiredFields<WritePositionRegisterPacket::Request>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("RegisterNumber");
}
template <>
inline bool hasRequiredFields<WritePositionRegisterPacket::Response>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("ErrorID") && document.contains("RegisterNumber");
}
template <>
inline bool hasRequiredFields<ResetRobotPacket::Request>(const nlohmann::json& document)
{
  return document.contains("Command");
}
template <>
inline bool hasRequiredFields<ResetRobotPacket::Response>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("ErrorID");
}
template <>
inline bool hasRequiredFields<GetTCPSpeedPacket::Request>(const nlohmann::json& document)
{
  return document.contains("Command");
}
template <>
inline bool hasRequiredFields<GetTCPSpeedPacket::Response>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("ErrorID") && document.contains("TimeTag") &&
         document.contains("Speed");
}
template <>
inline bool hasRequiredFields<WaitForDINPacket::Request>(const nlohmann::json& document)
{
  return document.contains("Instruction") && document.contains("SequenceID") && document.contains("PortNumber") &&
         document.contains("PortValue");
}
template <>
inline bool hasRequiredFields<WaitForDINPacket::Response>(const nlohmann::json& document)
{
  return document.contains("Instruction") && document.contains("ErrorID") && document.contains("SequenceID");
}
template <>
inline bool hasRequiredFields<SetUFramePacket::Request>(const nlohmann::json& document)
{
  return document.contains("Instruction") && document.contains("SequenceID") && document.contains("FrameNumber");
}
template <>
inline bool hasRequiredFields<SetUFramePacket::Response>(const nlohmann::json& document)
{
  return document.contains("Instruction") && document.contains("ErrorID") && document.contains("SequenceID");
}
template <>
inline bool hasRequiredFields<SetToolFramePacket::Request>(const nlohmann::json& document)
{
  return document.contains("Instruction") && document.contains("SequenceID") && document.contains("ToolNumber");
}
template <>
inline bool hasRequiredFields<SetToolFramePacket::Response>(const nlohmann::json& document)
{
  return document.contains("Instruction") && document.contains("ErrorID") && document.contains("SequenceID");
}
template <>
inline bool hasRequiredFields<WaitForTimePacket::Request>(const nlohmann::json& document)
{
  return document.contains("Instruction") && document.contains("SequenceID") && document.contains("Time");
}
template <>
inline bool hasRequiredFields<WaitForTimePacket::Response>(const nlohmann::json& document)
{
  return document.contains("Instruction") && document.contains("ErrorID") && document.contains("SequenceID");
}
template <>
inline bool hasRequiredFields<SetPayloadInstructionPacket::Request>(const nlohmann::json& document)
{
  return document.contains("Instruction") && document.contains("SequenceID") && document.contains("ScheduleNumber");
}
template <>
inline bool hasRequiredFields<SetPayloadInstructionPacket::Response>(const nlohmann::json& document)
{
  return document.contains("Instruction") && document.contains("ErrorID") && document.contains("SequenceID");
}
template <>
inline bool hasRequiredFields<SetPayloadPacket::Request>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("ScheduleNumber");
}
template <>
inline bool hasRequiredFields<SetPayloadPacket::Response>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("ErrorID");
}
template <>
inline bool hasRequiredFields<GetPayloadPacket::Request>(const nlohmann::json& document)
{
  return document.contains("Command");
}
template <>
inline bool hasRequiredFields<GetPayloadPacket::Response>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("ScheduleNumber") && document.contains("ErrorID");
}
template <>
inline bool hasRequiredFields<ProgramCallPacket::Request>(const nlohmann::json& document)
{
  return document.contains("Instruction") && document.contains("SequenceID") && document.contains("ProgramName");
}
template <>
inline bool hasRequiredFields<ProgramCallPacket::Response>(const nlohmann::json& document)
{
  return document.contains("Instruction") && document.contains("ErrorID") && document.contains("SequenceID");
}
template <>
inline bool hasRequiredFields<LinearMotionPacket::Request>(const nlohmann::json& document)
{
  return document.contains("Instruction") && document.contains("SequenceID") && document.contains("Configuration") &&
         document.contains("Position") && document.contains("SpeedType") && document.contains("Speed") &&
         document.contains("TermType") && document.contains("TermValue");
}
template <>
inline bool hasRequiredFields<LinearMotionPacket::Response>(const nlohmann::json& document)
{
  return document.contains("Instruction") && document.contains("ErrorID") && document.contains("SequenceID");
}
template <>
inline bool hasRequiredFields<LinearRelativePacket::Request>(const nlohmann::json& document)
{
  return document.contains("Instruction") && document.contains("SequenceID") && document.contains("Configuration") &&
         document.contains("Position") && document.contains("SpeedType") && document.contains("Speed") &&
         document.contains("TermType") && document.contains("TermValue");
}
template <>
inline bool hasRequiredFields<LinearRelativePacket::Response>(const nlohmann::json& document)
{
  return document.contains("Instruction") && document.contains("ErrorID") && document.contains("SequenceID");
}
template <>
inline bool hasRequiredFields<JointMotionPacket::Request>(const nlohmann::json& document)
{
  return document.contains("Instruction") && document.contains("SequenceID") && document.contains("Configuration") &&
         document.contains("Position") && document.contains("SpeedType") && document.contains("Speed") &&
         document.contains("TermType") && document.contains("TermValue");
}
template <>
inline bool hasRequiredFields<JointMotionPacket::Response>(const nlohmann::json& document)
{
  return document.contains("Instruction") && document.contains("ErrorID") && document.contains("SequenceID");
}
template <>
inline bool hasRequiredFields<JointRelativePacket::Request>(const nlohmann::json& document)
{
  return document.contains("Instruction") && document.contains("SequenceID") && document.contains("Configuration") &&
         document.contains("Position") && document.contains("SpeedType") && document.contains("Speed") &&
         document.contains("TermType") && document.contains("TermValue");
}
template <>
inline bool hasRequiredFields<JointRelativePacket::Response>(const nlohmann::json& document)
{
  return document.contains("Instruction") && document.contains("ErrorID") && document.contains("SequenceID");
}
template <>
inline bool hasRequiredFields<CircularMotionPacket::Request>(const nlohmann::json& document)
{
  return document.contains("Instruction") && document.contains("SequenceID") && document.contains("Configuration") &&
         document.contains("Position") && document.contains("ViaConfiguration") && document.contains("ViaPosition") &&
         document.contains("SpeedType") && document.contains("Speed") && document.contains("TermType") &&
         document.contains("TermValue");
}
template <>
inline bool hasRequiredFields<CircularMotionPacket::Response>(const nlohmann::json& document)
{
  return document.contains("Instruction") && document.contains("ErrorID") && document.contains("SequenceID");
}
template <>
inline bool hasRequiredFields<CircularRelativePacket::Request>(const nlohmann::json& document)
{
  return document.contains("Instruction") && document.contains("SequenceID") && document.contains("Configuration") &&
         document.contains("Position") && document.contains("ViaConfiguration") && document.contains("ViaPosition") &&
         document.contains("SpeedType") && document.contains("Speed") && document.contains("TermType") &&
         document.contains("TermValue");
}
template <>
inline bool hasRequiredFields<CircularRelativePacket::Response>(const nlohmann::json& document)
{
  return document.contains("Instruction") && document.contains("ErrorID") && document.contains("SequenceID");
}
template <>
inline bool hasRequiredFields<JointMotionJRepPacket::Request>(const nlohmann::json& document)
{
  return document.contains("Instruction") && document.contains("SequenceID") && document.contains("JointAngle") &&
         document.contains("SpeedType") && document.contains("Speed") && document.contains("TermType") &&
         document.contains("TermValue");
}
template <>
inline bool hasRequiredFields<JointMotionJRepPacket::Response>(const nlohmann::json& document)
{
  return document.contains("Instruction") && document.contains("ErrorID") && document.contains("SequenceID");
}
template <>
inline bool hasRequiredFields<JointRelativeJRepPacket::Request>(const nlohmann::json& document)
{
  return document.contains("Instruction") && document.contains("SequenceID") && document.contains("JointAngle") &&
         document.contains("SpeedType") && document.contains("Speed") && document.contains("TermType") &&
         document.contains("TermValue");
}
template <>
inline bool hasRequiredFields<JointRelativeJRepPacket::Response>(const nlohmann::json& document)
{
  return document.contains("Instruction") && document.contains("ErrorID") && document.contains("SequenceID");
}
template <>
inline bool hasRequiredFields<LinearMotionJRepPacket::Request>(const nlohmann::json& document)
{
  return document.contains("Instruction") && document.contains("SequenceID") && document.contains("JointAngle") &&
         document.contains("SpeedType") && document.contains("Speed") && document.contains("TermType") &&
         document.contains("TermValue");
}
template <>
inline bool hasRequiredFields<LinearMotionJRepPacket::Response>(const nlohmann::json& document)
{
  return document.contains("Instruction") && document.contains("ErrorID") && document.contains("SequenceID");
}
template <>
inline bool hasRequiredFields<LinearRelativeJRepPacket::Request>(const nlohmann::json& document)
{
  return document.contains("Instruction") && document.contains("SequenceID") && document.contains("JointAngle") &&
         document.contains("SpeedType") && document.contains("Speed") && document.contains("TermType") &&
         document.contains("TermValue");
}
template <>
inline bool hasRequiredFields<LinearRelativeJRepPacket::Response>(const nlohmann::json& document)
{
  return document.contains("Instruction") && document.contains("ErrorID") && document.contains("SequenceID");
}
template <>
inline bool hasRequiredFields<SplineMotionPacket::Request>(const nlohmann::json& document)
{
  return document.contains("Instruction") && document.contains("SequenceID") && document.contains("Configuration") &&
         document.contains("Position") && document.contains("SpeedType") && document.contains("Speed") &&
         document.contains("TermType") && document.contains("TermValue");
}
template <>
inline bool hasRequiredFields<SplineMotionPacket::Response>(const nlohmann::json& document)
{
  return document.contains("Instruction") && document.contains("ErrorID") && document.contains("SequenceID");
}
template <>
inline bool hasRequiredFields<SplineMotionJRepPacket::Request>(const nlohmann::json& document)
{
  return document.contains("Instruction") && document.contains("SequenceID") && document.contains("JointAngle") &&
         document.contains("SpeedType") && document.contains("Speed") && document.contains("TermType") &&
         document.contains("TermValue");
}
template <>
inline bool hasRequiredFields<SplineMotionJRepPacket::Response>(const nlohmann::json& document)
{
  return document.contains("Instruction") && document.contains("ErrorID") && document.contains("SequenceID");
}
template <>
inline bool hasRequiredFields<UnknownPacket>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("ErrorID");
}
template <>
inline bool hasRequiredFields<CommunicationPacket>(const nlohmann::json& document)
{
  return document.contains("Communication");
}
template <>
inline bool hasRequiredFields<ReadNumericRegisterPacket::Request>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("RegisterNumber");
}
template <>
inline bool hasRequiredFields<ReadNumericRegisterPacket::Response>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("ErrorID") && document.contains("DataType") &&
         document.contains("RegisterValue");
}
template <>
inline bool hasRequiredFields<WriteNumericRegisterPacket::Request>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("RegisterNumber") && document.contains("RegisterValue") &&
         document.contains("DataType");
}
template <>
inline bool hasRequiredFields<WriteNumericRegisterPacket::Response>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("ErrorID");
}
template <>
inline bool hasRequiredFields<ReadIOPortPacket::Request>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("PortType") && document.contains("PortNumber");
}
template <>
inline bool hasRequiredFields<ReadIOPortPacket::Response>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("ErrorID") && document.contains("PortType") &&
         document.contains("PortNumber") && document.contains("PortValue");
}
template <>
inline bool hasRequiredFields<WriteIOPortPacket::Request>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("PortType") && document.contains("PortNumber") &&
         document.contains("PortValue");
}
template <>
inline bool hasRequiredFields<WriteIOPortPacket::Response>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("ErrorID") && document.contains("PortType") &&
         document.contains("PortNumber");
}
template <>
inline bool hasRequiredFields<ReadVariablePacket::Request>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("VariableName");
}
template <>
inline bool hasRequiredFields<ReadVariablePacket::Response>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("ErrorID") && document.contains("VariableName") &&
         document.contains("VariableType") && document.contains("VariableValue");
}
template <>
inline bool hasRequiredFields<WriteVariablePacket::Request>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("VariableName") && document.contains("VariableType") &&
         document.contains("VariableValue");
}
template <>
inline bool hasRequiredFields<WriteVariablePacket::Response>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("ErrorID") && document.contains("VariableName");
}
template <>
inline bool hasRequiredFields<GetExtendedStatusPacket::Request>(const nlohmann::json& document)
{
  return document.contains("Command");
}
template <>
inline bool hasRequiredFields<GetExtendedStatusPacket::Response>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("ErrorID") && document.contains("InMotion") &&
         document.contains("DrivesPowered") && document.contains("GenOverride");
}
template <>
inline bool hasRequiredFields<SetPayloadValuePacket::Request>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("ScheduleNumber") && document.contains("Mass") &&
         document.contains("CG_X") && document.contains("CG_Y") && document.contains("CG_Z");
}
template <>
inline bool hasRequiredFields<SetPayloadValuePacket::Response>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("ErrorID");
}
template <>
inline bool hasRequiredFields<GetPayloadValuePacket::Request>(const nlohmann::json& document)
{
  return document.contains("Command");
}
template <>
inline bool hasRequiredFields<GetPayloadValuePacket::Response>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("Mass") && document.contains("CG_X") &&
         document.contains("CG_Y") && document.contains("CG_Z") && document.contains("IN_X") &&
         document.contains("IN_Y") && document.contains("IN_Z") && document.contains("ErrorID");
}
template <>
inline bool hasRequiredFields<SetPayloadCompPacket::Request>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("ScheduleNumber") && document.contains("Mass") &&
         document.contains("CG_X") && document.contains("CG_Y") && document.contains("CG_Z") &&
         document.contains("IN_X") && document.contains("IN_Y") && document.contains("IN_Z");
}
template <>
inline bool hasRequiredFields<SetPayloadCompPacket::Response>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("ErrorID");
}
template <>
inline bool hasRequiredFields<GetPayloadCompPacket::Request>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("ScheduleNumber");
}
template <>
inline bool hasRequiredFields<GetPayloadCompPacket::Response>(const nlohmann::json& document)
{
  return document.contains("Command") && document.contains("Mass") && document.contains("CG_X") &&
         document.contains("CG_Y") && document.contains("CG_Z") && document.contains("IN_X") &&
         document.contains("IN_Y") && document.contains("IN_Z") && document.contains("ErrorID");
}
}  // namespace rmi::json_detail
