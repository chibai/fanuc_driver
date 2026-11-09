// SPDX-FileCopyrightText: 2025, FANUC America Corporation
// SPDX-FileCopyrightText: 2025, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include "drivers/fanuc/detail/line_buffer.hpp"
#include "drivers/fanuc/detail/serialization.hpp"
#include "robot_driver/drivers/fanuc/robot.hpp"

TEST(ToJSON, ConnectPacket)
{
  const std::string expected_connect_packet_json = R"({"Communication":"FRC_Connect"})";
  const rmi::ConnectPacket::Request connect_packet;
  const std::string connect_packet_json = rmi::ToJSON(connect_packet);
  EXPECT_EQ(connect_packet_json, expected_connect_packet_json);
}

TEST(ToJSON, OmitsUnsetOptionalFields)
{
  const rmi::InitializePacket::Request initialize_packet;

  EXPECT_EQ(rmi::ToJSON(initialize_packet), R"({"Command":"FRC_Initialize"})");
}

TEST(FromJSON, ConnectPacketResponse)
{
  rmi::ConnectPacket::Response expected_connect_packet_response{};
  expected_connect_packet_response.ErrorID = 0;
  expected_connect_packet_response.PortNumber = 10;
  expected_connect_packet_response.MajorVersion = 3;
  expected_connect_packet_response.MinorVersion = 1;

  const std::string connect_packet_response_json =
      R"({"Communication":"FRC_Connect","ErrorID":0,"PortNumber":10,"MajorVersion":3,"MinorVersion":1})";
  const auto connect_packet_response = rmi::FromJSON<rmi::ConnectPacket::Response>(connect_packet_response_json);

  EXPECT_EQ(expected_connect_packet_response.Communication, connect_packet_response.value().Communication);
  EXPECT_EQ(expected_connect_packet_response.ErrorID, connect_packet_response.value().ErrorID);
  EXPECT_EQ(expected_connect_packet_response.MajorVersion, connect_packet_response.value().MajorVersion);
  EXPECT_EQ(expected_connect_packet_response.MinorVersion, connect_packet_response.value().MinorVersion);
  EXPECT_EQ(expected_connect_packet_response.PortNumber, connect_packet_response.value().PortNumber);
}

TEST(FromJSON, RejectsInvalidAndNonInstructionPackets)
{
  EXPECT_FALSE(rmi::FromJSON<rmi::ConnectPacket::Response>("not json"));
  EXPECT_FALSE(rmi::FromJSON<rmi::ConnectPacket::Response>(R"({"Communication":"FRC_Connect"})"));
  EXPECT_FALSE(rmi::IsInstruction(R"({"Command":"FRC_Reset","ErrorID":0})"));
}

TEST(FromJSON, ErrorResponsesDoNotRequireSuccessData)
{
  const auto error =
      rmi::FromJSON<rmi::ConnectPacket::Response>(R"({"Communication":"FRC_Connect","ErrorID":2556954})");
  ASSERT_TRUE(error);
  EXPECT_EQ(error->ErrorID, 2556954);
  EXPECT_FALSE(rmi::FromJSON<rmi::ConnectPacket::Response>(R"({"Communication":"FRC_Disconnect","ErrorID":2556954})"));
  EXPECT_FALSE(rmi::FromJSON<rmi::ConnectPacket::Response>(R"({"Communication":"FRC_Connect","ErrorID":0})"));
  EXPECT_FALSE(rmi::FromJSON<rmi::ConnectPacket::Response>(R"({"Communication":"FRC_Connect","ErrorID":"invalid"})"));
  EXPECT_FALSE(
      rmi::FromJSON<rmi::JointMotionPacket::Response>(R"({"Instruction":"FRC_JointMotion","ErrorID":2556959})"));
  const auto instruction = rmi::FromJSON<rmi::JointMotionPacket::Response>(
      R"({"Instruction":"FRC_JointMotion","ErrorID":2556959,"SequenceID":7})");
  ASSERT_TRUE(instruction);
  EXPECT_EQ(instruction->ErrorID, 2556959);
  EXPECT_EQ(instruction->SequenceID, 7);
  const auto unknown = rmi::FromJSON<rmi::UnknownPacket>(R"({"Command":"Unknown","ErrorID":42})");
  ASSERT_TRUE(unknown);
  EXPECT_FALSE(unknown->SequenceID);
  const auto correlated = rmi::FromJSON<rmi::UnknownPacket>(R"({"Command":"Unknown","ErrorID":42,"SequenceID":9})");
  ASSERT_TRUE(correlated);
  EXPECT_EQ(correlated->SequenceID, 9);
}

TEST(RmiLineBuffer, PreservesPartialFramesAcrossAppends)
{
  rmi::detail::LineBuffer buffer;

  EXPECT_TRUE(buffer.append(R"({"first":1)").empty());
  const auto lines = buffer.append("}\r\n");

  ASSERT_EQ(lines.size(), 1);
  EXPECT_EQ(lines.front(), R"({"first":1})");
}

TEST(RmiLineBuffer, ReturnsEveryCompleteFrameAndKeepsTrailingFragment)
{
  rmi::detail::LineBuffer buffer;

  const auto first_lines = buffer.append("one\r\ntwo\r\npar");
  ASSERT_EQ(first_lines.size(), 2);
  EXPECT_EQ(first_lines[0], "one");
  EXPECT_EQ(first_lines[1], "two");

  const auto second_lines = buffer.append("tial\r\n");
  ASSERT_EQ(second_lines.size(), 1);
  EXPECT_EQ(second_lines.front(), "partial");
}
