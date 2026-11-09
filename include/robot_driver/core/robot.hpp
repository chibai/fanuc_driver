// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>
#include <expected>
#include <optional>

#include "robot_driver/core/error.hpp"
#include "robot_driver/core/types.hpp"

class Robot_driver
{
public:
  virtual ~Robot_driver() = default;

  Robot_driver(const Robot_driver&) = delete;
  Robot_driver& operator=(const Robot_driver&) = delete;

  ResponseStatus connect(std::optional<double> timeout = std::nullopt)
  {
    return doConnect(timeout);
  }

  ResponseStatus disconnect(std::optional<double> timeout = std::nullopt)
  {
    return doDisconnect(timeout);
  }

  ResponseStatus initializeRemoteMotion(std::optional<double> timeout = std::nullopt)
  {
    return doInitializeRemoteMotion(timeout);
  }

  ResponseStatus pause(std::optional<double> timeout = std::nullopt)
  {
    return doPause(timeout);
  }

  ResponseStatus resume(std::optional<double> timeout = std::nullopt)
  {
    return doResume(timeout);
  }

  ResponseStatus abort(std::optional<double> timeout = std::nullopt)
  {
    return doAbort(timeout);
  }

  ResponseStatus reset(std::optional<double> timeout = std::nullopt)
  {
    return doReset(timeout);
  }

  ResponseStatus setSpeedOverride(int value, std::optional<double> timeout = std::nullopt)
  {
    return doSetSpeedOverride(value, timeout);
  }

  Response<RobotStatus> getStatus(std::optional<double> timeout = std::nullopt)
  {
    return doGetStatus(timeout);
  }

  std::expected<bool, ResponseStatus> readDigitalInputPort(uint16_t port_number,
                                                           std::optional<double> timeout = std::nullopt)
  {
    return doReadDigitalInputPort(port_number, timeout);
  }

  ResponseStatus writeDigitalOutputPort(uint16_t port_number, bool port_value,
                                        std::optional<double> timeout = std::nullopt)
  {
    return doWriteDigitalOutputPort(port_number, port_value, timeout);
  }

protected:
  Robot_driver() = default;

  virtual ResponseStatus doConnect(std::optional<double> timeout) = 0;
  virtual ResponseStatus doDisconnect(std::optional<double> timeout) = 0;
  virtual ResponseStatus doInitializeRemoteMotion(std::optional<double> timeout) = 0;
  virtual ResponseStatus doPause(std::optional<double> timeout) = 0;
  virtual ResponseStatus doResume(std::optional<double> timeout) = 0;
  virtual ResponseStatus doAbort(std::optional<double> timeout) = 0;
  virtual ResponseStatus doReset(std::optional<double> timeout) = 0;
  virtual ResponseStatus doSetSpeedOverride(int value, std::optional<double> timeout) = 0;
  virtual Response<RobotStatus> doGetStatus(std::optional<double> timeout) = 0;
  virtual std::expected<bool, ResponseStatus> doReadDigitalInputPort(uint16_t port_number,
                                                                     std::optional<double> timeout) = 0;
  virtual ResponseStatus doWriteDigitalOutputPort(uint16_t port_number, bool port_value,
                                                  std::optional<double> timeout) = 0;
};
