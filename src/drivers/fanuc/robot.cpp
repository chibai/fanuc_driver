// SPDX-FileCopyrightText: 2025-2026, FANUC America Corporation
// SPDX-FileCopyrightText: 2025-2026, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#include "robot_driver/drivers/fanuc/robot.hpp"
#include "detail/line_buffer.hpp"
#include "detail/response.hpp"
#include "detail/serialization.hpp"

#include <iconv.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <expected>
#include <iostream>
#include <iterator>
#include <mutex>
#include <ranges>
#include <span>
#include <sstream>
#include <system_error>
#include <utility>

#include <boost/asio.hpp>
#include "robot_driver/drivers/fanuc/packets.hpp"

using namespace rmi;

namespace
{
constexpr auto kEndChars = "\r\n";

constexpr std::array<std::pair<uint32_t, std::string_view>, 55> kErrorCodes = {
  { { 2556929, "Internal System Error." },
    { 2556930, "Invalid UTool Number." },
    { 2556931, "Invalid UFrame Number." },
    { 2556932, "Invalid Position Register." },
    { 2556933, "Invalid Speed Override." },
    { 2556934, "Cannot Execute TP program." },
    { 2556935, "Controller Servo is Off." },
    { 2556936, "Cannot Execute TP program." },
    { 2556937, "RMI is Not Running." },
    { 2556938, "TP Program is Not Paused." },
    { 2556939, "Cannot Resume TP Program." },
    { 2556940, "Cannot Reset Controller." },
    { 2556941, "Invalid RMI Command." },
    { 2556942, "RMI Command Fail." },
    { 2556943, "Invalid Controller State." },
    { 2556944, "Please Cycle Power." },
    { 2556945, "Invalid Payload Schedule." },
    { 2556946, "Invalid Motion Option." },
    { 2556947, "Invalid Vision Register." },
    { 2556948, "Invalid RMI Instruction." },
    { 2556949, "Invalid Value." },
    { 2556950, "Invalid Text String" },
    { 2556951, "Invalid Position Data" },
    { 2556952, "RMI is In HOLD State" },
    { 2556953, "Remote Device Disconnected." },
    { 2556954, "Robot is Already Connected." },
    { 2556955, "Wait for Command Done." },
    { 2556956, "Wait for Instruction Done." },
    { 2556957, "Invalid sequence ID number." },
    { 2556958, "Invalid Speed Type." },
    { 2556959, "Invalid Speed Value." },
    { 2556960, "Invalid Term Type." },
    { 2556961, "Invalid Term Value." },
    { 2556962, "Invalid LCB Port Type." },
    { 2556963, "Invalid ACC Value." },
    { 2556964, "Invalid Destination Position" },
    { 2556965, "Invalid VIA Position." },
    { 2556966, "Invalid Port Number." },
    { 2556967, "Invalid Group Number" },
    { 2556968, "Invalid Group Mask" },
    { 2556969, "Joint motion with COORD" },
    { 2556970, "Incremental motn with COORD" },
    { 2556971, "Robot in Single Step Mode" },
    { 2556972, "Invalid Position Data Type" },
    { 2556973, "Ready for ASCII Packet" },
    { 2556974, "ASCII Conversion Failed" },
    { 2556975, "Invalid ASCII Instruction" },
    { 2556976, "Invalid Number of Groups" },
    { 2556977, "Invalid Instruction packet" },
    { 2556978, "Invalid ASCII String packet" },
    { 2556979, "Invalid ASCII string size" },
    { 2556980, "Invalid Application Tool" },
    { 2556981, "Invalid Call Program Name" },
    { 7015, "RMI_MOVE is the selected TP program. Select a different TP program and re-run RMI PC code." },
    { 7004, "The specific program is in use." } }
};

std::string LookupErrorCode(const uint32_t error_code)
{
  if (const auto it = std::ranges::find(kErrorCodes, error_code, &decltype(kErrorCodes)::value_type::first);
      it != kErrorCodes.end())
  {
    return std::string(it->second);
  }
  return "Unknown Error Code: " + std::to_string(error_code);
}

template <typename T>
bool CheckSequenceIDIfPresent(T& packet, int expected_sequence_id)
{
  if constexpr (requires(T t) { t.SequenceID; })
  {
    return packet.SequenceID == expected_sequence_id;
  }
  return true;
}

std::string convertEncoding(const std::string& input, const char* from, const char* to)
{
  iconv_t cd = iconv_open(to, from);
  if (cd == (iconv_t)-1)
  {
    throw rmi::detail::ResponseError({ ResponseCode::SERVER_ERR, "iconv_open failed", std::nullopt });
  }

  size_t inBytes = input.size();
  size_t outBytes = inBytes * 4;
  std::string output(outBytes, '\0');

  char* inBuf = const_cast<char*>(input.data());
  char* outBuf = output.data();

  iconv(cd, &inBuf, &inBytes, &outBuf, &outBytes);

  iconv_close(cd);

  output.resize(output.size() - outBytes);
  return output;
}

void addCallArguments(ProgramCallPacket::Request& packet, const std::span<const RMICallParam> params)
{
  using Request = ProgramCallPacket::Request;
  constexpr std::array fields{
    std::pair{ &Request::ParamType1, &Request::ParamValue1 }, std::pair{ &Request::ParamType2, &Request::ParamValue2 },
    std::pair{ &Request::ParamType3, &Request::ParamValue3 }, std::pair{ &Request::ParamType4, &Request::ParamValue4 },
    std::pair{ &Request::ParamType5, &Request::ParamValue5 }, std::pair{ &Request::ParamType6, &Request::ParamValue6 },
    std::pair{ &Request::ParamType7, &Request::ParamValue7 }, std::pair{ &Request::ParamType8, &Request::ParamValue8 },
    std::pair{ &Request::ParamType9, &Request::ParamValue9 }, std::pair{ &Request::ParamType10, &Request::ParamValue10 }
  };
  if (params.size() > fields.size())
    throw std::invalid_argument("RMI program calls support at most 10 arguments.");
  for (const auto& [param, field] : std::views::zip(params, fields))
  {
    packet.*field.first = param.first;
    packet.*field.second = param.second;
  }
}

}  // namespace

struct Fanuc_driver::PConnectionImpl
{
  using Clock = std::chrono::steady_clock;
  using IOResult = std::expected<std::size_t, boost::system::error_code>;

  PConnectionImpl(const std::string& robot_ip_address, const uint16_t robot_port)
  {
    createNewConnection(robot_ip_address, robot_port);
  }

  template <typename T>
  void write(const T& value, const std::string& encoding = "UTF-8")
  {
    const std::string json_string = [&]() {
      std::string json_str = ToJSON(value) + kEndChars;
      if constexpr (std::is_same_v<T, ProgramCallPacket::Request>)
      {
        if (encoding != "UTF-8")
        {
          json_str = convertEncoding(json_str, "UTF-8", encoding.c_str());
        }
      }
      return json_str;
    }();
    std::scoped_lock lock(mutex_);
    std::optional<IOResult> result;
    boost::asio::async_write(socket_, boost::asio::buffer(json_string),
                             [&result](const boost::system::error_code error, const std::size_t size) {
                               result = error ? IOResult(std::unexpected(error)) : IOResult(size);
                             });
    context_.restart();
    while (!result)
      context_.run_one();
    if (!*result)
      throw rmi::detail::ResponseError({ ResponseCode::CONNECTION_ERROR,
                                         "Failed to write to RMI TCP connection: " + result->error().message(),
                                         std::nullopt });
  }

  void createNewConnection(const std::string& robot_ip_address, const uint16_t robot_port)
  {
    std::scoped_lock lock(mutex_);
    boost::asio::ip::tcp::resolver resolver(context_);
    boost::system::error_code error;
    const auto endpoints = resolver.resolve(robot_ip_address, std::to_string(robot_port), error);
    if (error)
      throw rmi::detail::ResponseError({ ResponseCode::CONNECTION_ERROR,
                                         "Failed to resolve RMI server " + robot_ip_address + ": " + error.message(),
                                         std::nullopt });

    // Drain cancellation handlers before replacing buffers used by the old session.
    if (socket_.is_open())
    {
      socket_.close(error);
      if (error)
        throw rmi::detail::ResponseError(
            { ResponseCode::CONNECTION_ERROR, "Failed to close RMI TCP connection: " + error.message(), std::nullopt });
    }
    context_.restart();
    context_.run();
    read_result_.reset();
    read_pending_ = false;
    line_buffer_ = {};

    boost::asio::connect(socket_, endpoints, error);
    if (error)
      throw rmi::detail::ResponseError({ ResponseCode::CONNECTION_ERROR,
                                         "Failed to create TCP connection at " + robot_ip_address + ":" +
                                             std::to_string(robot_port) + ": " + error.message(),
                                         std::nullopt });
  }

  std::vector<std::string> read()
  {
    std::scoped_lock lock(mutex_);
    std::vector<std::string> lines;
    startRead();
    context_.restart();
    context_.poll();
    while (read_result_)
    {
      if (!*read_result_)
      {
        // Deliver complete frames before reporting a following EOF on the next call.
        if (!lines.empty())
          return lines;
        if (read_result_->error() == boost::asio::error::eof)
          throw rmi::detail::ResponseError(
              { ResponseCode::CONNECTION_ERROR, "RMI TCP connection was closed by the controller.", std::nullopt });
        throw rmi::detail::ResponseError({ ResponseCode::CONNECTION_ERROR,
                                           "Failed to read from RMI TCP connection: " + read_result_->error().message(),
                                           std::nullopt });
      }
      auto frames = line_buffer_.append(std::string_view(read_buffer_.data(), read_result_->value()));
      lines.insert(lines.end(), std::make_move_iterator(frames.begin()), std::make_move_iterator(frames.end()));
      read_result_.reset();
      startRead();
      context_.restart();
      context_.poll();
    }
    return lines;
  }

  void waitForData(const std::optional<Clock::time_point> deadline)
  {
    std::scoped_lock lock(mutex_);
    startRead();
    if (read_result_)
      return;
    context_.restart();
    if (deadline)
      context_.run_one_until(*deadline);
    else
      context_.run_one();
  }

private:
  void startRead()
  {
    if (read_pending_ || read_result_)
      return;
    read_pending_ = true;
    socket_.async_read_some(boost::asio::buffer(read_buffer_),
                            [this](const boost::system::error_code error, const std::size_t size) {
                              read_pending_ = false;
                              read_result_ = error ? IOResult(std::unexpected(error)) : IOResult(size);
                            });
  }

  boost::asio::io_context context_;
  boost::asio::ip::tcp::socket socket_{ context_ };
  std::mutex mutex_;
  std::array<char, 2048> read_buffer_{};
  bool read_pending_ = false;
  std::optional<IOResult> read_result_;
  rmi::detail::LineBuffer line_buffer_;
};

Fanuc_driver::Fanuc_driver(const std::string& robot_ip_address, const uint16_t rmi_port)
  : robot_ip_address_{ robot_ip_address }
  , rmi_port_{ rmi_port }
  , encoding_{ "UTF-8" }
  , sequence_number_{ 1 }
  , connection_impl_{ std::make_unique<PConnectionImpl>(robot_ip_address, rmi_port) }
  , last_instruction_response_{ std::nullopt }
{
}

Fanuc_driver::~Fanuc_driver() = default;

void Fanuc_driver::drainConnectionBuffer()
{
  auto json_responses = connection_impl_->read();
  std::scoped_lock lock(mutex_);
  json_responses_.insert(json_responses_.end(), std::make_move_iterator(json_responses.begin()),
                         std::make_move_iterator(json_responses.end()));
}

bool Fanuc_driver::processInstructionResponse(std::string& json_response)
{
  const auto json = IsInstruction(json_response);

  if (!json.has_value())
  {
    return false;  // not instruction
  }

  const int sequence_id = json.value().SequenceID;

  std::scoped_lock lock(instruction_mutex_);
  if (!last_instruction_response_.has_value() || (last_instruction_response_.value().SequenceID < sequence_id))
  {
    last_instruction_response_ = std::optional<InstructionResponse>(json.value());
  }
  return true;
}

template <typename T>
std::optional<T> Fanuc_driver::checkForPacketInJSONResponses(std::list<std::string>::iterator& it,
                                                             std::list<std::string>& json_responses,
                                                             const std::optional<int> expected_sequence_id)
{
  std::optional<T> packet_response;
  for (; it != json_responses.end();)
  {
    if constexpr (std::is_same_v<T, ReadErrorPacket::Response>)
    {
      if (encoding_ == "SHIFT-JIS")
      {
        // Japanese R-30iB Plus controllers send alarm text in SHIFT-JIS.
        const auto converted = convertEncoding(*it, encoding_.c_str(), "UTF-8");
        if (auto response = FromJSON<T>(converted); response.has_value())
        {
          packet_response = std::move(response);
          it = json_responses.erase(it);
          break;
        }
      }
    }
    // Match the awaited reply before consuming asynchronous instruction acknowledgements.
    if (auto response = FromJSON<T>(*it); response.has_value())
    {
      processInstructionResponse(*it);
      if (expected_sequence_id && !CheckSequenceIDIfPresent(*response, *expected_sequence_id))
      {
        it = json_responses.erase(it);
        continue;
      }
      packet_response = std::move(response);
      it = json_responses.erase(it);
      break;
    }
    if (processInstructionResponse(*it))
    {
      it = json_responses.erase(it);
      continue;
    }
    ++it;
  }
  return packet_response;
}

template <typename T>
std::optional<T> Fanuc_driver::checkPushPacket()
{
  drainConnectionBuffer();

  std::scoped_lock lock(mutex_);
  std::optional<T> packet;
  auto it = json_responses_.begin();
  auto possible_packet = checkForPacketInJSONResponses<T>(it, json_responses_);
  while (possible_packet.has_value())
  {
    packet = possible_packet;
    possible_packet = checkForPacketInJSONResponses<T>(it, json_responses_);
  }

  return packet;
}

std::optional<SystemFaultPacket> Fanuc_driver::checkSystemFault()
{
  return checkPushPacket<SystemFaultPacket>();
}

std::optional<TimeoutTerminatePacket> Fanuc_driver::checkTimeoutTerminate()
{
  return checkPushPacket<TimeoutTerminatePacket>();
}

std::optional<CommunicationPacket> Fanuc_driver::checkCommunicationPacket()
{
  return checkPushPacket<CommunicationPacket>();
}

std::optional<UnknownPacket> Fanuc_driver::checkUnknownPacket()
{
  return checkPushPacket<UnknownPacket>();
}

std::optional<InstructionResponse> Fanuc_driver::getLastInstructionResponse()
{
  drainConnectionBuffer();
  {
    std::scoped_lock lock(mutex_);
    for (auto it = json_responses_.begin(); it != json_responses_.end();)
    {
      if (processInstructionResponse(*it))
        it = json_responses_.erase(it);
      else
        ++it;
    }
  }
  std::scoped_lock lock(instruction_mutex_);
  return last_instruction_response_;
}

template <typename T>
T Fanuc_driver::getResponsePacket(const std::optional<double> timeout_optional, const std::string& error_message_prefix,
                                  std::optional<int> expected_sequence_id)
{
  if constexpr (requires(T t) { t.SequenceID; })
  {
    if (!expected_sequence_id.has_value())
    {
      throw std::logic_error("expected_sequence_id must be provided for the packet type T with the SequenceID field.");
    }
  }

  const double timeout = timeout_optional.value_or(1.0);
  if (!std::isfinite(timeout))
    throw std::invalid_argument("RMI response timeout must be finite; use a negative value for unlimited waiting.");
  std::optional<T> packet_response;

  const auto start_time = std::chrono::steady_clock::now();
  const auto deadline = [&]() -> std::optional<PConnectionImpl::Clock::time_point> {
    if (timeout < 0.0)
      return std::nullopt;
    const auto max_wait = PConnectionImpl::Clock::time_point::max() - start_time;
    if (timeout >= std::chrono::duration<double>(max_wait).count())
      return PConnectionImpl::Clock::time_point::max();
    return start_time +
           std::chrono::duration_cast<PConnectionImpl::Clock::duration>(std::chrono::duration<double>(timeout));
  }();
  while (!packet_response.has_value() ||
         (expected_sequence_id.has_value() &&
          !CheckSequenceIDIfPresent(packet_response.value(), expected_sequence_id.value())))
  {
    if (const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time);
        timeout >= 0.0 && elapsed.count() >= timeout)
    {
      std::stringstream ss;
      ss << error_message_prefix << "Timeout waiting for response. ";
      std::scoped_lock lock(mutex_);
      if (!json_responses_.empty())
      {
        ss << " The current unhandled json response packets are:\n";
        for (const auto& json_response : json_responses_)
        {
          ss << json_response << "\n";
        }
        ss << "You may have incompatible packet definitions for the current RMI version.";
      }
      throw rmi::detail::ResponseError({ ResponseCode::WAITING_TIMEOUT, ss.str(), std::nullopt });
    }

    drainConnectionBuffer();
    {
      std::scoped_lock lock(mutex_);
      if (auto it = json_responses_.begin(); !json_responses_.empty())
        packet_response = checkForPacketInJSONResponses<T>(it, json_responses_, expected_sequence_id);
    }
    if (!packet_response || (expected_sequence_id && !CheckSequenceIDIfPresent(*packet_response, *expected_sequence_id)))
      connection_impl_->waitForData(deadline);
  }

  if (packet_response.value().ErrorID != 0)
  {
    throw rmi::detail::ResponseError(
        { ResponseCode::CONTROLLER_ERROR,
          error_message_prefix + "Error: " + LookupErrorCode(packet_response.value().ErrorID),
          packet_response.value().ErrorID });
  }

  return packet_response.value();
}

int32_t Fanuc_driver::getSequenceNumber()
{
  return sequence_number_++;
}

ResponseStatus Fanuc_driver::doConnect(std::optional<double> timeout)
{
  return connect(timeout).status;
}

ResponseStatus Fanuc_driver::doDisconnect(std::optional<double> timeout)
{
  return disconnect(timeout).status;
}

ResponseStatus Fanuc_driver::doInitializeRemoteMotion(std::optional<double> timeout)
{
  return initializeRemoteMotion(timeout).status;
}

ResponseStatus Fanuc_driver::doPause(std::optional<double> timeout)
{
  return pause(timeout).status;
}

ResponseStatus Fanuc_driver::doResume(std::optional<double> timeout)
{
  return resume(timeout).status;
}

ResponseStatus Fanuc_driver::doAbort(std::optional<double> timeout)
{
  return abort(timeout).status;
}

ResponseStatus Fanuc_driver::doReset(std::optional<double> timeout)
{
  return reset(timeout).status;
}

ResponseStatus Fanuc_driver::doSetSpeedOverride(int value, std::optional<double> timeout)
{
  return setSpeedOverride(value, timeout).status;
}

Response<RobotStatus> Fanuc_driver::doGetStatus(std::optional<double> timeout)
{
  const auto status = getStatus(timeout);
  if (!status.ok())
    return { status.status, std::nullopt };
  return { status.status, RobotStatus{ status.data->ServoReady != 0, status.data->Override } };
}

std::expected<bool, ResponseStatus> Fanuc_driver::doReadDigitalInputPort(uint16_t port_number,
                                                                         std::optional<double> timeout)
{
  const auto response = readDigitalInputPort(port_number, timeout);
  if (!response)
    return std::unexpected(response.error());
  return response->PortValue != 0;
}

ResponseStatus Fanuc_driver::doWriteDigitalOutputPort(uint16_t port_number, bool port_value,
                                                      std::optional<double> timeout)
{
  return writeDigitalOutputPort(port_number, port_value, timeout).status;
}

Response<ConnectPacket::Response> Fanuc_driver::connect(const std::optional<double> timeout)
{
  return rmi::detail::captureResponse<ConnectPacket::Response>([&]() {
    connection_impl_->write(ConnectPacket::Request());
    auto connect_response =
        getResponsePacket<ConnectPacket::Response>(timeout, "Failed to connect to RMI server. ", std::nullopt);
    // Recreate a TCP to the new port number provided by the server
    connection_impl_->createNewConnection(robot_ip_address_, connect_response.PortNumber);
    return connect_response;
  });
}

Response<DisconnectPacket::Response> Fanuc_driver::disconnect(const std::optional<double> timeout)
{
  return rmi::detail::captureResponse<DisconnectPacket::Response>([&]() {
    connection_impl_->write(DisconnectPacket::Request());
    return getResponsePacket<DisconnectPacket::Response>(timeout, "Failed to disconnect from RMI server. ",
                                                         std::nullopt);
  });
}

Response<InitializePacket::Response> Fanuc_driver::initializeRemoteMotion(const std::optional<double> timeout)
{
  return initializeRemoteMotion(timeout, std::nullopt, std::nullopt, std::nullopt);
}

Response<InitializePacket::Response> Fanuc_driver::initializeRemoteMotion(const std::optional<double> timeout,
                                                                          const std::optional<uint8_t> groupmask,
                                                                          const std::optional<std::string>& rtsa,
                                                                          const std::optional<std::string>& pltzmode)
{
  return rmi::detail::captureResponse<InitializePacket::Response>([&]() {
    {
      std::scoped_lock lock(mutex_);
      sequence_number_ = 1;  // Reset sequence number for a new session
    }
    {
      std::scoped_lock lock(instruction_mutex_);
      last_instruction_response_ = std::nullopt;
    }

    const auto packet = [&]() {
      auto request = InitializePacket::Request();
      request.GroupMask = groupmask;
      request.RTSA = rtsa;
      request.PLTZMODE = pltzmode;
      return request;
    }();
    connection_impl_->write(packet);

    return getResponsePacket<InitializePacket::Response>(timeout, "Failed to initialize RMI. ", std::nullopt);
  });
}

ProgramCallPacket::Request Fanuc_driver::programCallNonBlocking(const std::string& program_name)
{
  return programCallNonBlocking(program_name, {});
}

ProgramCallPacket::Request Fanuc_driver::programCallNonBlocking(const std::string& program_name,
                                                                const std::vector<RMICallParam>& params)
{
  ProgramCallPacket::Request program_call_packet;
  program_call_packet.ProgramName = program_name;
  addCallArguments(program_call_packet, params);
  std::scoped_lock lock(motion_mutex_);
  program_call_packet.SequenceID = getSequenceNumber();
  connection_impl_->write(program_call_packet, encoding_);
  return program_call_packet;
}

Response<ProgramCallPacket::Response> Fanuc_driver::programCall(const std::string& program_name,
                                                                const std::optional<double> timeout)
{
  return programCall(program_name, timeout, {});
}

Response<ProgramCallPacket::Response> Fanuc_driver::programCall(const std::string& program_name,
                                                                std::optional<double> timeout,
                                                                const std::vector<RMICallParam>& params)
{
  return rmi::detail::captureResponse<ProgramCallPacket::Response>([&]() {
    ProgramCallPacket::Request program_call_packet = programCallNonBlocking(program_name, params);
    return getResponsePacket<ProgramCallPacket::Response>(
        timeout, "Failed to call program named `" + program_name + "`. ", program_call_packet.SequenceID);
  });
}

Response<SetSpeedOverridePacket::Response> Fanuc_driver::setSpeedOverride(const int value,
                                                                          const std::optional<double> timeout)
{
  return rmi::detail::captureResponse<SetSpeedOverridePacket::Response>([&]() {
    SetSpeedOverridePacket::Request set_speed_override_packet;
    set_speed_override_packet.Value = value;
    connection_impl_->write(set_speed_override_packet);
    return getResponsePacket<SetSpeedOverridePacket::Response>(timeout, "Failed to set the override speed. ",
                                                               std::nullopt);
  });
}

Response<StatusRequestPacket::Response> Fanuc_driver::getStatus(const std::optional<double> timeout)
{
  return rmi::detail::captureResponse<StatusRequestPacket::Response>([&]() {
    connection_impl_->write(StatusRequestPacket::Request());
    return getResponsePacket<StatusRequestPacket::Response>(timeout, "Failed to get status from RMI server. ",
                                                            std::nullopt);
  });
}

Response<AbortPacket::Response> Fanuc_driver::abort(const std::optional<double> timeout)
{
  return rmi::detail::captureResponse<AbortPacket::Response>([&]() {
    connection_impl_->write(AbortPacket::Request());
    return getResponsePacket<AbortPacket::Response>(timeout, "Failed to abort. ", std::nullopt);
  });
}

Response<PausePacket::Response> Fanuc_driver::pause(const std::optional<double> timeout)
{
  return rmi::detail::captureResponse<PausePacket::Response>([&]() {
    connection_impl_->write(PausePacket::Request());
    return getResponsePacket<PausePacket::Response>(timeout, "Failed to pause. ", std::nullopt);
  });
}

Response<ContinuePacket::Response> Fanuc_driver::resume(const std::optional<double> timeout)
{
  return rmi::detail::captureResponse<ContinuePacket::Response>([&]() {
    connection_impl_->write(ContinuePacket::Request());
    return getResponsePacket<ContinuePacket::Response>(timeout, "Failed to continue. ", std::nullopt);
  });
}

Response<ResetRobotPacket::Response> Fanuc_driver::reset(const std::optional<double> timeout)
{
  return rmi::detail::captureResponse<ResetRobotPacket::Response>([&]() {
    connection_impl_->write(ResetRobotPacket::Request());
    return getResponsePacket<ResetRobotPacket::Response>(timeout, "Failed to reset. ", std::nullopt);
  });
}

Response<ReadErrorPacket::Response> Fanuc_driver::readError(const std::optional<double> timeout)
{
  return readError(timeout, std::nullopt);
}

Response<ReadErrorPacket::Response> Fanuc_driver::readError(const std::optional<double> timeout,
                                                            const std::optional<uint8_t> count)
{
  return rmi::detail::captureResponse<ReadErrorPacket::Response>([&]() {
    const auto packet = [&]() {
      auto request = ReadErrorPacket::Request();
      request.Count = count;
      return request;
    }();
    connection_impl_->write(packet);
    return getResponsePacket<ReadErrorPacket::Response>(timeout, "Failed to read error. ", std::nullopt);
  });
}

Response<GetUFrameToolFramePacket::Response> Fanuc_driver::getUFrameUTool(std::optional<double> timeout,
                                                                          const std::optional<uint8_t> group)
{
  auto request = GetUFrameToolFramePacket::Request();
  request.Group = group;

  return sendRMIPacket<GetUFrameToolFramePacket>(request, timeout);
}

Response<SetUFrameToolFramePacket::Response> Fanuc_driver::setUFrameUTool(const int uframe, const int utool,
                                                                          std::optional<double> timeout,
                                                                          const std::optional<uint8_t> group)
{
  auto request = SetUFrameToolFramePacket::Request();
  request.UFrameNumber = uframe;
  request.UToolNumber = utool;
  request.Group = group;

  return sendRMIPacket<SetUFrameToolFramePacket>(request, timeout);
}

Response<ReadUFrameDataPacket::Response> Fanuc_driver::readUFrameData(const int uframe, std::optional<double> timeout,
                                                                      const std::optional<uint8_t> group)
{
  auto request = ReadUFrameDataPacket::Request();
  request.FrameNumber = uframe;
  request.Group = group;

  return sendRMIPacket<ReadUFrameDataPacket>(request, timeout);
}

Response<WriteUFrameDataPacket::Response> Fanuc_driver::writeUFrameData(const int uframe, const FrameData data,
                                                                        std::optional<double> timeout,
                                                                        const std::optional<uint8_t> group)
{
  auto request = WriteUFrameDataPacket::Request();
  request.FrameNumber = uframe;
  request.Frame = data;
  request.Group = group;

  return sendRMIPacket<WriteUFrameDataPacket>(request, timeout);
}

Response<ReadUToolDataPacket::Response> Fanuc_driver::readUToolData(const int utool, std::optional<double> timeout,
                                                                    const std::optional<uint8_t> group)
{
  auto request = ReadUToolDataPacket::Request();
  request.ToolNumber = utool;
  request.Group = group;

  return sendRMIPacket<ReadUToolDataPacket>(request, timeout);
}

Response<WriteUToolDataPacket::Response> Fanuc_driver::writeUToolData(const int utool, const FrameData data,
                                                                      std::optional<double> timeout,
                                                                      const std::optional<uint8_t> group)
{
  auto request = WriteUToolDataPacket::Request();
  request.ToolNumber = utool;
  request.Frame = data;
  request.Group = group;

  return sendRMIPacket<WriteUToolDataPacket>(request, timeout);
}

Response<WritePositionRegisterPacket::Response>
Fanuc_driver::writePositionRegister(const int register_number, const std::string& representation,
                                    const ConfigurationData& configuration, const PositionData& position,
                                    const JointAngleData& joint_angle, const std::optional<double> timeout)
{
  return rmi::detail::captureResponse<WritePositionRegisterPacket::Response>([&]() {
    if (register_number < 1 || register_number > 100)
    {
      throw rmi::detail::ResponseError(
          { ResponseCode::INDEX_OUT_OF_RANGE,
            "Invalid position register number: " + std::to_string(register_number) + ". Valid range is 1 to 100.",
            std::nullopt });
    }
    WritePositionRegisterPacket::Request write_position_register_packet;
    write_position_register_packet.RegisterNumber = register_number;
    write_position_register_packet.Representation = representation;
    std::string temp_upper_rep = representation;
    std::transform(temp_upper_rep.begin(), temp_upper_rep.end(), temp_upper_rep.begin(), ::toupper);
    if (temp_upper_rep == "JOINT")
    {
      write_position_register_packet.JointAngle = joint_angle;
    }
    else
    {
      write_position_register_packet.Configuration = configuration;
      write_position_register_packet.Position = position;
    }
    connection_impl_->write(write_position_register_packet);
    return getResponsePacket<WritePositionRegisterPacket::Response>(timeout, "Failed to write to position register. ",
                                                                    std::nullopt);
  });
}

std::expected<ReadPositionRegisterPacket::Response, ResponseStatus>
Fanuc_driver::readPositionRegister(const int register_number, const std::optional<double> timeout)
{
  return rmi::detail::captureExpected<ReadPositionRegisterPacket::Response>([&]() {
    if (register_number < 1 || register_number > 100)
    {
      throw rmi::detail::ResponseError(
          { ResponseCode::INDEX_OUT_OF_RANGE,
            "Invalid position register number: " + std::to_string(register_number) + ". Valid range is 1 to 100.",
            std::nullopt });
    }
    ReadPositionRegisterPacket::Request read_position_register_packet;
    read_position_register_packet.RegisterNumber = register_number;
    connection_impl_->write(read_position_register_packet);
    return getResponsePacket<ReadPositionRegisterPacket::Response>(timeout, "Failed to read from position register. ",
                                                                   std::nullopt);
  });
}

std::expected<ReadNumericRegisterPacket::Response, ResponseStatus>
Fanuc_driver::readNumericRegister(const int register_number, const std::optional<double> timeout)
{
  return rmi::detail::captureExpected<ReadNumericRegisterPacket::Response>([&]() {
    if (register_number < 1 || register_number > 200)
    {
      throw rmi::detail::ResponseError(
          { ResponseCode::INDEX_OUT_OF_RANGE,
            "Invalid numeric register number: " + std::to_string(register_number) + ". Valid range is 1 to 200.",
            std::nullopt });
    }
    ReadNumericRegisterPacket::Request read_numeric_register_packet;
    read_numeric_register_packet.RegisterNumber = register_number;
    connection_impl_->write(read_numeric_register_packet);
    return getResponsePacket<ReadNumericRegisterPacket::Response>(timeout, "Failed to read from numeric register. ",
                                                                  std::nullopt);
  });
}

Response<WriteNumericRegisterPacket::Response> Fanuc_driver::writeNumericRegister(const int register_number,
                                                                                  const std::variant<int, float> value,
                                                                                  const std::optional<double> timeout)
{
  return rmi::detail::captureResponse<WriteNumericRegisterPacket::Response>([&]() {
    if (register_number < 1 || register_number > 200)
    {
      throw rmi::detail::ResponseError(
          { ResponseCode::INDEX_OUT_OF_RANGE,
            "Invalid numeric register number: " + std::to_string(register_number) + ". Valid range is 1 to 200.",
            std::nullopt });
    }
    WriteNumericRegisterPacket::Request write_numeric_register_packet;
    write_numeric_register_packet.RegisterNumber = register_number;
    write_numeric_register_packet.RegisterValue = value;
    write_numeric_register_packet.DataType = std::holds_alternative<float>(value) ? "float" : "integer";
    connection_impl_->write(write_numeric_register_packet);
    return getResponsePacket<WriteNumericRegisterPacket::Response>(timeout, "Failed to write to numeric register. ",
                                                                   std::nullopt);
  });
}

std::expected<ReadDigitalInputPortPacket::Response, ResponseStatus>
Fanuc_driver::readDigitalInputPort(const uint16_t port_number, const std::optional<double> timeout)
{
  return rmi::detail::captureExpected<ReadDigitalInputPortPacket::Response>([&]() {
    ReadDigitalInputPortPacket::Request read_digital_input_port_packet;
    read_digital_input_port_packet.PortNumber = port_number;
    connection_impl_->write(read_digital_input_port_packet);
    return getResponsePacket<ReadDigitalInputPortPacket::Response>(timeout, "Failed to read digital input port. ",
                                                                   std::nullopt);
  });
}

Response<WriteDigitalOutputPacket::Response> Fanuc_driver::writeDigitalOutputPort(const uint16_t port_number,
                                                                                  const bool port_value,
                                                                                  const std::optional<double> timeout)
{
  return rmi::detail::captureResponse<WriteDigitalOutputPacket::Response>([&]() {
    WriteDigitalOutputPacket::Request write_digital_output_packet;
    write_digital_output_packet.PortNumber = port_number;
    write_digital_output_packet.PortValue = port_value ? "ON" : "OFF";
    connection_impl_->write(write_digital_output_packet);
    return getResponsePacket<WriteDigitalOutputPacket::Response>(timeout, "Failed to write to digital output port. ",
                                                                 std::nullopt);
  });
}

std::expected<ReadIOPortPacket::Response, ResponseStatus>
Fanuc_driver::readIOPort(const std::string& port_type, const int port_number, const std::optional<double> timeout)
{
  return rmi::detail::captureExpected<ReadIOPortPacket::Response>([&]() {
    ReadIOPortPacket::Request read_io_port_packet;
    read_io_port_packet.PortNumber = port_number;
    read_io_port_packet.PortType = port_type;
    connection_impl_->write(read_io_port_packet);
    return getResponsePacket<ReadIOPortPacket::Response>(timeout, "Failed to read IO port. ", std::nullopt);
  });
}

Response<WriteIOPortPacket::Response> Fanuc_driver::writeIOPort(const int port_number, const std::string& port_type,
                                                                const std::variant<int, float> port_value,
                                                                const std::optional<double> timeout)
{
  return rmi::detail::captureResponse<WriteIOPortPacket::Response>([&]() {
    WriteIOPortPacket::Request write_io_port_packet;
    write_io_port_packet.PortNumber = port_number;
    write_io_port_packet.PortType = port_type;
    write_io_port_packet.PortValue = port_value;
    connection_impl_->write(write_io_port_packet);
    return getResponsePacket<WriteIOPortPacket::Response>(timeout, "Failed to read IO port. ", std::nullopt);
  });
}

Response<ReadVariablePacket::Response> Fanuc_driver::readVariablePacket(const std::string& variable_name,
                                                                        const std::optional<double> timeout)
{
  return rmi::detail::captureResponse<ReadVariablePacket::Response>([&]() {
    ReadVariablePacket::Request read_variable_packet;
    read_variable_packet.VariableName = variable_name;
    connection_impl_->write(read_variable_packet);
    return getResponsePacket<ReadVariablePacket::Response>(timeout, "Failed to read variable `" + variable_name + "`. ",
                                                           std::nullopt);
  });
}

Response<WriteVariablePacket::Response> Fanuc_driver::writeVariablePacket(const std::string& variable_name,
                                                                          const std::variant<int, float> value,
                                                                          const std::optional<double> timeout)
{
  return rmi::detail::captureResponse<WriteVariablePacket::Response>([&]() {
    WriteVariablePacket::Request write_variable_packet;
    write_variable_packet.VariableName = variable_name;
    write_variable_packet.VariableValue = value;
    write_variable_packet.VariableType = std::holds_alternative<float>(value) ? "float" : "integer";
    connection_impl_->write(write_variable_packet);
    return getResponsePacket<WriteVariablePacket::Response>(
        timeout, "Failed to write variable `" + variable_name + "`. ", std::nullopt);
  });
}

Response<GetExtendedStatusPacket::Response> Fanuc_driver::getExtendedStatus(const std::optional<double> timeout)
{
  return rmi::detail::captureResponse<GetExtendedStatusPacket::Response>([&]() {
    connection_impl_->write(GetExtendedStatusPacket::Request());
    return getResponsePacket<GetExtendedStatusPacket::Response>(timeout, "Failed to get extended status. ",
                                                                std::nullopt);
  });
}

Response<SetPayloadPacket::Response> Fanuc_driver::setPayloadSchedule(const uint8_t payload_schedule_number,
                                                                      const std::optional<double> timeout)
{
  return rmi::detail::captureResponse<SetPayloadPacket::Response>([&]() {
    SetPayloadPacket::Request set_payload_packet;
    set_payload_packet.ScheduleNumber = payload_schedule_number;
    std::scoped_lock lock(motion_mutex_);
    connection_impl_->write(set_payload_packet);
    return getResponsePacket<SetPayloadPacket::Response>(timeout, "Failed to set the payload schedule. ", std::nullopt);
  });
}

Response<SetPayloadValuePacket::Response>
Fanuc_driver::setPayloadValue(const uint8_t payload_schedule_number, const float mass, const float cg_x,
                              const float cg_y, const float cg_z, const bool use_in, const float in_x, const float in_y,
                              const float in_z, const std::optional<double> timeout)
{
  return rmi::detail::captureResponse<SetPayloadValuePacket::Response>([&]() {
    SetPayloadValuePacket::Request set_payload_value_packet;
    set_payload_value_packet.ScheduleNumber = payload_schedule_number;
    set_payload_value_packet.Mass = mass;
    set_payload_value_packet.CG_X = cg_x * 100;  // unit conversion: m -> cm (rmi)
    set_payload_value_packet.CG_Y = cg_y * 100;  // unit conversion: m -> cm (rmi)
    set_payload_value_packet.CG_Z = cg_z * 100;  // unit conversion: m -> cm (rmi)
    if (use_in)
    {
      set_payload_value_packet.IN_X = in_x * 10000;  // unit conversion: kgm^2 -> kgcm^2 (rmi)
      set_payload_value_packet.IN_Y = in_y * 10000;  // unit conversion: kgm^2 -> kgcm^2 (rmi)
      set_payload_value_packet.IN_Z = in_z * 10000;  // unit conversion: kgm^2 -> kgcm^2 (rmi)
    }
    std::scoped_lock lock(motion_mutex_);
    connection_impl_->write(set_payload_value_packet);
    return getResponsePacket<SetPayloadValuePacket::Response>(timeout, "Failed to set the payload value. ",
                                                              std::nullopt);
  });
}

Response<SetPayloadCompPacket::Response>
Fanuc_driver::setPayloadComp(const uint8_t payload_schedule_number, const float mass, const float cg_x,
                             const float cg_y, const float cg_z, const float in_x, const float in_y, const float in_z,
                             const std::optional<double> timeout)
{
  return rmi::detail::captureResponse<SetPayloadCompPacket::Response>([&]() {
    SetPayloadCompPacket::Request set_payload_comp_packet;
    set_payload_comp_packet.ScheduleNumber = payload_schedule_number;
    set_payload_comp_packet.Mass = mass;
    set_payload_comp_packet.CG_X = cg_x * 100;    // unit conversion: m -> cm (rmi)
    set_payload_comp_packet.CG_Y = cg_y * 100;    // unit conversion: m -> cm (rmi)
    set_payload_comp_packet.CG_Z = cg_z * 100;    // unit conversion: m -> cm (rmi)
    set_payload_comp_packet.IN_X = in_x * 10000;  // unit conversion: kgm^2 -> kgcm^2 (rmi)
    set_payload_comp_packet.IN_Y = in_y * 10000;  // unit conversion: kgm^2 -> kgcm^2 (rmi)
    set_payload_comp_packet.IN_Z = in_z * 10000;  // unit conversion: kgm^2 -> kgcm^2 (rmi)
    std::scoped_lock lock(motion_mutex_);
    connection_impl_->write(set_payload_comp_packet);
    return getResponsePacket<SetPayloadCompPacket::Response>(timeout, "Failed to set the payload compensation. ",
                                                             std::nullopt);
  });
}

Response<GetPayloadPacket::Response> Fanuc_driver::getPayloadSchedule(const std::optional<double> timeout)
{
  return rmi::detail::captureResponse<GetPayloadPacket::Response>([&]() {
    GetPayloadPacket::Request get_payload_packet;
    std::scoped_lock lock(motion_mutex_);
    connection_impl_->write(get_payload_packet);
    return getResponsePacket<GetPayloadPacket::Response>(timeout, "Failed to get the payload schedule. ", std::nullopt);
  });
}

Response<GetPayloadValuePacket::Response> Fanuc_driver::getPayloadValue(const uint16_t payload_schedule_number,
                                                                        const std::optional<double> timeout)
{
  return rmi::detail::captureResponse<GetPayloadValuePacket::Response>([&]() {
    GetPayloadValuePacket::Request get_payload_value_packet;
    get_payload_value_packet.ScheduleNumber = payload_schedule_number;
    std::scoped_lock lock(motion_mutex_);
    connection_impl_->write(get_payload_value_packet);
    return getResponsePacket<GetPayloadValuePacket::Response>(timeout, "Failed to get the payload value. ",
                                                              std::nullopt);
  });
}

Response<GetPayloadCompPacket::Response> Fanuc_driver::getPayloadComp(const uint16_t payload_schedule_number,
                                                                      const std::optional<double> timeout)
{
  return rmi::detail::captureResponse<GetPayloadCompPacket::Response>([&]() {
    GetPayloadCompPacket::Request get_payload_comp_packet;
    get_payload_comp_packet.ScheduleNumber = payload_schedule_number;
    std::scoped_lock lock(motion_mutex_);
    connection_impl_->write(get_payload_comp_packet);
    return getResponsePacket<GetPayloadCompPacket::Response>(timeout, "Failed to get the payload compensation. ",
                                                             std::nullopt);
  });
}

Response<ReadJointAnglesPacket::Response> Fanuc_driver::readJointAngles(const std::optional<uint8_t>& group,
                                                                        const std::optional<double> timeout)
{
  return rmi::detail::captureResponse<ReadJointAnglesPacket::Response>([&]() {
    ReadJointAnglesPacket::Request read_joint_angles_packet;
    read_joint_angles_packet.Group = group;
    connection_impl_->write(read_joint_angles_packet);
    return getResponsePacket<ReadJointAnglesPacket::Response>(timeout, "Failed to get joint angles. ", std::nullopt);
  });
}

Response<GetCartesianPositionPacket::Response> Fanuc_driver::getCartesianPosition(std::optional<double> timeout,
                                                                                  const std::optional<uint8_t> group)
{
  return rmi::detail::captureResponse<GetCartesianPositionPacket::Response>([&]() {
    GetCartesianPositionPacket::Request packet;
    packet.Group = group;
    connection_impl_->write(packet);
    return getResponsePacket<GetCartesianPositionPacket::Response>(timeout, "Failed to get cartesian position. ",
                                                                   std::nullopt);
  });
}

Response<GetTCPSpeedPacket::Response> Fanuc_driver::getTCPSpeed(std::optional<double> timeout,
                                                                const std::optional<uint8_t> group)
{
  return rmi::detail::captureResponse<GetTCPSpeedPacket::Response>([&]() {
    GetTCPSpeedPacket::Request packet;
    packet.Group = group;
    connection_impl_->write(packet);
    return getResponsePacket<GetTCPSpeedPacket::Response>(timeout, "Failed to get TCP speed. ", std::nullopt);
  });
}

Response<JointMotionJRepPacket::Response>
Fanuc_driver::sendJointMotion(JointMotionJRepPacket::Request joint_motion_request, const std::optional<double> timeout)
{
  return rmi::detail::captureResponse<JointMotionJRepPacket::Response>([&]() {
    joint_motion_request.SequenceID = getSequenceNumber();
    connection_impl_->write(joint_motion_request);
    return getResponsePacket<JointMotionJRepPacket::Response>(timeout, "Failed to send joint motion. ",
                                                              joint_motion_request.SequenceID);
  });
}

void Fanuc_driver::sendRMIPacketNonBlocking(JointMotionPacket::Request& packet)
{
  sendRMIPacketNonBlockingImpl<JointMotionPacket>(packet);
  return;
};
void Fanuc_driver::sendRMIPacketNonBlocking(JointMotionJRepPacket::Request& packet)
{
  sendRMIPacketNonBlockingImpl<JointMotionJRepPacket>(packet);
  return;
};
void Fanuc_driver::sendRMIPacketNonBlocking(JointRelativePacket::Request& packet)
{
  sendRMIPacketNonBlockingImpl<JointRelativePacket>(packet);
  return;
};
void Fanuc_driver::sendRMIPacketNonBlocking(JointRelativeJRepPacket::Request& packet)
{
  sendRMIPacketNonBlockingImpl<JointRelativeJRepPacket>(packet);
  return;
};
void Fanuc_driver::sendRMIPacketNonBlocking(LinearMotionPacket::Request& packet)
{
  sendRMIPacketNonBlockingImpl<LinearMotionPacket>(packet);
  return;
};
void Fanuc_driver::sendRMIPacketNonBlocking(LinearMotionJRepPacket::Request& packet)
{
  sendRMIPacketNonBlockingImpl<LinearMotionJRepPacket>(packet);
  return;
};
void Fanuc_driver::sendRMIPacketNonBlocking(LinearRelativePacket::Request& packet)
{
  sendRMIPacketNonBlockingImpl<LinearRelativePacket>(packet);
  return;
};
void Fanuc_driver::sendRMIPacketNonBlocking(LinearRelativeJRepPacket::Request& packet)
{
  sendRMIPacketNonBlockingImpl<LinearRelativeJRepPacket>(packet);
  return;
};
void Fanuc_driver::sendRMIPacketNonBlocking(CircularMotionPacket::Request& packet)
{
  sendRMIPacketNonBlockingImpl<CircularMotionPacket>(packet);
  return;
};
void Fanuc_driver::sendRMIPacketNonBlocking(CircularRelativePacket::Request& packet)
{
  sendRMIPacketNonBlockingImpl<CircularRelativePacket>(packet);
  return;
};
void Fanuc_driver::sendRMIPacketNonBlocking(SplineMotionPacket::Request& packet)
{
  sendRMIPacketNonBlockingImpl<SplineMotionPacket>(packet);
  return;
};
void Fanuc_driver::sendRMIPacketNonBlocking(SplineMotionJRepPacket::Request& packet)
{
  sendRMIPacketNonBlockingImpl<SplineMotionJRepPacket>(packet);
  return;
};

void Fanuc_driver::sendRMIPacketNonBlocking(WaitForDINPacket::Request& packet)
{
  sendRMIPacketNonBlockingImpl<WaitForDINPacket>(packet);
  return;
};
void Fanuc_driver::sendRMIPacketNonBlocking(SetUFramePacket::Request& packet)
{
  sendRMIPacketNonBlockingImpl<SetUFramePacket>(packet);
  return;
};
void Fanuc_driver::sendRMIPacketNonBlocking(SetToolFramePacket::Request& packet)
{
  sendRMIPacketNonBlockingImpl<SetToolFramePacket>(packet);
  return;
};
void Fanuc_driver::sendRMIPacketNonBlocking(WaitForTimePacket::Request& packet)
{
  sendRMIPacketNonBlockingImpl<WaitForTimePacket>(packet);
  return;
};
void Fanuc_driver::sendRMIPacketNonBlocking(SetPayloadInstructionPacket::Request& packet)
{
  sendRMIPacketNonBlockingImpl<SetPayloadInstructionPacket>(packet);
  return;
};

std::string Fanuc_driver::getErrorMessageString(const uint32_t error_code)
{
  return LookupErrorCode(error_code);
}

int32_t Fanuc_driver::getRemainingBufferSize()
{
  constexpr int32_t kMaxBufferSize = 8;
  getLastInstructionResponse();
  std::scoped_lock lock(instruction_mutex_);
  const auto acknowledged = last_instruction_response_ ? last_instruction_response_->SequenceID : 0;
  return kMaxBufferSize - (sequence_number_ - 1 - acknowledged);
}

void Fanuc_driver::setEncoding(const std::string& encoding)
{
  encoding_ = encoding;
}

template <typename T>
Response<typename T::Response> Fanuc_driver::sendRMIPacket(typename T::Request& request_packet,
                                                           const std::optional<double> timeout)
{
  return rmi::detail::captureResponse<typename T::Response>([&]() {
    if constexpr (requires(typename T::Response t) { t.SequenceID; })
    {
      std::scoped_lock lock(motion_mutex_);
      request_packet.SequenceID = getSequenceNumber();
      connection_impl_->write(request_packet);
      return getResponsePacket<typename T::Response>(timeout, "Failed to send packet. ", request_packet.SequenceID);
    }

    connection_impl_->write(request_packet);
    return getResponsePacket<typename T::Response>(timeout, "Failed to send packet. ", std::nullopt);
  });
}

template <typename T>
void Fanuc_driver::sendRMIPacketNonBlockingImpl(typename T::Request& request_packet)
{
  if constexpr (requires(typename T::Response t) { t.SequenceID; })
  {
    std::scoped_lock lock(motion_mutex_);
    request_packet.SequenceID = getSequenceNumber();
    connection_impl_->write(request_packet);
    return;
  }

  connection_impl_->write(request_packet);
  return;
}

template Response<SetUFrameToolFramePacket::Response>
Fanuc_driver::sendRMIPacket<SetUFrameToolFramePacket>(SetUFrameToolFramePacket::Request&, std::optional<double>);
template Response<ReadUFrameDataPacket::Response>
Fanuc_driver::sendRMIPacket<ReadUFrameDataPacket>(ReadUFrameDataPacket::Request&, std::optional<double>);
template Response<WriteUFrameDataPacket::Response>
Fanuc_driver::sendRMIPacket<WriteUFrameDataPacket>(WriteUFrameDataPacket::Request&, std::optional<double>);
template Response<ReadUToolDataPacket::Response>
Fanuc_driver::sendRMIPacket<ReadUToolDataPacket>(ReadUToolDataPacket::Request&, std::optional<double>);
template Response<WriteUToolDataPacket::Response>
Fanuc_driver::sendRMIPacket<WriteUToolDataPacket>(WriteUToolDataPacket::Request&, std::optional<double>);
template Response<WriteDigitalOutputPacket::Response>
Fanuc_driver::sendRMIPacket<WriteDigitalOutputPacket>(WriteDigitalOutputPacket::Request&, std::optional<double>);
template Response<ReadJointAnglesPacket::Response>
Fanuc_driver::sendRMIPacket<ReadJointAnglesPacket>(ReadJointAnglesPacket::Request&, std::optional<double>);
template Response<GetUFrameToolFramePacket::Response>
Fanuc_driver::sendRMIPacket<GetUFrameToolFramePacket>(GetUFrameToolFramePacket::Request&, std::optional<double>);
template Response<GetTCPSpeedPacket::Response>
Fanuc_driver::sendRMIPacket<GetTCPSpeedPacket>(GetTCPSpeedPacket::Request&, std::optional<double>);
template Response<WaitForDINPacket::Response> Fanuc_driver::sendRMIPacket<WaitForDINPacket>(WaitForDINPacket::Request&,
                                                                                            std::optional<double>);
template Response<SetUFramePacket::Response> Fanuc_driver::sendRMIPacket<SetUFramePacket>(SetUFramePacket::Request&,
                                                                                          std::optional<double>);
template Response<SetToolFramePacket::Response>
Fanuc_driver::sendRMIPacket<SetToolFramePacket>(SetToolFramePacket::Request&, std::optional<double>);
template Response<WaitForTimePacket::Response>
Fanuc_driver::sendRMIPacket<WaitForTimePacket>(WaitForTimePacket::Request&, std::optional<double>);
template Response<LinearMotionPacket::Response>
Fanuc_driver::sendRMIPacket<LinearMotionPacket>(LinearMotionPacket::Request&, std::optional<double>);
template Response<LinearRelativePacket::Response>
Fanuc_driver::sendRMIPacket<LinearRelativePacket>(LinearRelativePacket::Request&, std::optional<double>);
template Response<JointMotionPacket::Response>
Fanuc_driver::sendRMIPacket<JointMotionPacket>(JointMotionPacket::Request&, std::optional<double>);
template Response<JointRelativePacket::Response>
Fanuc_driver::sendRMIPacket<JointRelativePacket>(JointRelativePacket::Request&, std::optional<double>);
template Response<CircularMotionPacket::Response>
Fanuc_driver::sendRMIPacket<CircularMotionPacket>(CircularMotionPacket::Request&, std::optional<double>);
template Response<CircularRelativePacket::Response>
Fanuc_driver::sendRMIPacket<CircularRelativePacket>(CircularRelativePacket::Request&, std::optional<double>);
template Response<JointMotionJRepPacket::Response>
Fanuc_driver::sendRMIPacket<JointMotionJRepPacket>(JointMotionJRepPacket::Request&, std::optional<double>);
template Response<JointRelativeJRepPacket::Response>
Fanuc_driver::sendRMIPacket<JointRelativeJRepPacket>(JointRelativeJRepPacket::Request&, std::optional<double>);
template Response<LinearMotionJRepPacket::Response>
Fanuc_driver::sendRMIPacket<LinearMotionJRepPacket>(LinearMotionJRepPacket::Request&, std::optional<double>);
template Response<LinearRelativeJRepPacket::Response>
Fanuc_driver::sendRMIPacket<LinearRelativeJRepPacket>(LinearRelativeJRepPacket::Request&, std::optional<double>);
template Response<SplineMotionPacket::Response>
Fanuc_driver::sendRMIPacket<SplineMotionPacket>(SplineMotionPacket::Request&, std::optional<double>);
template Response<SplineMotionJRepPacket::Response>
Fanuc_driver::sendRMIPacket<SplineMotionJRepPacket>(SplineMotionJRepPacket::Request&, std::optional<double>);
template Response<ConnectPacket::Response> Fanuc_driver::sendRMIPacket<ConnectPacket>(ConnectPacket::Request&,
                                                                                      std::optional<double>);
template Response<SetPayloadInstructionPacket::Response>
Fanuc_driver::sendRMIPacket<SetPayloadInstructionPacket>(SetPayloadInstructionPacket::Request&, std::optional<double>);
