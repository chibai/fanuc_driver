// SPDX-FileCopyrightText: 2025, FANUC America Corporation
// SPDX-FileCopyrightText: 2025, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include "rmi/rmi.hpp"
#include "rmi/detail/line_buffer.hpp"
#include "rmi/serialization.hpp"

TEST(ToJSON, ConnectPacket)
{
  const std::string expected_connect_packet_json = R"({Communication:"FRC_Connect}")";
  const rmi::ConnectPacket::Request connect_packet;
  const std::string connect_packet_json = rmi::ToJSON(connect_packet);
  EXPECT_EQ(expected_connect_packet_json, expected_connect_packet_json);
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
