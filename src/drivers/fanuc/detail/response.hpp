// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <expected>
#include <stdexcept>
#include <utility>

#include <boost/system/system_error.hpp>

#include "robot_driver/core/error.hpp"

namespace rmi::detail
{
class ResponseError : public std::runtime_error
{
public:
  explicit ResponseError(ResponseStatus status) : std::runtime_error(status.detail), status_(std::move(status))
  {
  }

  const ResponseStatus& status() const
  {
    return status_;
  }

private:
  ResponseStatus status_;
};

template <typename T, typename Operation>
std::expected<T, ResponseStatus> captureExpected(Operation&& operation)
{
  try
  {
    return std::forward<Operation>(operation)();
  }
  catch (const ResponseError& error)
  {
    return std::unexpected(error.status());
  }
  catch (const std::invalid_argument& error)
  {
    return std::unexpected(ResponseStatus{ ResponseCode::INVALID_PARAMETER, error.what() });
  }
  catch (const boost::system::system_error& error)
  {
    return std::unexpected(ResponseStatus{ ResponseCode::CONNECTION_ERROR, error.what() });
  }
}

template <typename T, typename Operation>
Response<T> captureResponse(Operation&& operation)
{
  auto result = captureExpected<T>(std::forward<Operation>(operation));
  if (!result)
    return { std::move(result.error()), std::nullopt };
  return { {}, std::move(*result) };
}
}  // namespace rmi::detail
