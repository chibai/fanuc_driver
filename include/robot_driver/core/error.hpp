// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <optional>
#include <string>
#include <string_view>

enum class ResponseCode : int
{
  OK = 0,
  INCOMPATIBLE_VERSION = 1110,
  CONNECTION_ERROR = 1120,
  CONNECTION_TIMEOUT = 1121,
  INVALID_IP_ADDRESS = 1122,
  INTERFACE_NOTIMPLEMENTED = 1140,
  UNSUPPORTED_PARAMETER = 1150,
  INVALID_PARAMETER = 1160,
  INDEX_OUT_OF_RANGE = 1901,
  CONTROLLER_ERROR = 3000,
  WAITING_TIMEOUT = 4411,
  SERVER_ERR = 9998
};

struct [[nodiscard]] ResponseStatus
{
  ResponseCode value = ResponseCode::OK;
  std::string detail;
  std::optional<int> controller_error;

  [[nodiscard]] int code() const
  {
    return static_cast<int>(value);
  }

  [[nodiscard]] bool ok() const
  {
    return value == ResponseCode::OK;
  }

  // Empty language uses ROBOTDRIVERCORE_LANGUAGE; unsupported values fall back to Chinese.
  [[nodiscard]] std::string_view message(std::string_view language = {}) const;
};

template <typename T>
struct [[nodiscard]] Response
{
  ResponseStatus status;
  std::optional<T> data;

  [[nodiscard]] bool ok() const
  {
    return status.ok() && data.has_value();
  }
};
