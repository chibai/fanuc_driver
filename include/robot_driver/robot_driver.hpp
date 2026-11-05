// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>
#include <optional>

struct RobotStatus
{
  bool servo_ready;
  int speed_override;  // Percent.
};

class Robot_driver
{
public:
  virtual ~Robot_driver() = default;

  Robot_driver(const Robot_driver&) = delete;
  Robot_driver& operator=(const Robot_driver&) = delete;

  // Non-virtual entry points allow drivers to retain their native response types.
  void connect(std::optional<double> timeout = std::nullopt)
  {
    doConnect(timeout);
  }

  void disconnect(std::optional<double> timeout = std::nullopt)
  {
    doDisconnect(timeout);
  }

  void initializeRemoteMotion(std::optional<double> timeout = std::nullopt)
  {
    doInitializeRemoteMotion(timeout);
  }

  void pause(std::optional<double> timeout = std::nullopt)
  {
    doPause(timeout);
  }

  void resume(std::optional<double> timeout = std::nullopt)
  {
    doResume(timeout);
  }

  void abort(std::optional<double> timeout = std::nullopt)
  {
    doAbort(timeout);
  }

  void reset(std::optional<double> timeout = std::nullopt)
  {
    doReset(timeout);
  }

  void setSpeedOverride(int value, std::optional<double> timeout = std::nullopt)
  {
    doSetSpeedOverride(value, timeout);
  }

  RobotStatus getStatus(std::optional<double> timeout = std::nullopt)
  {
    return doGetStatus(timeout);
  }

  bool readDigitalInputPort(uint16_t port_number, std::optional<double> timeout = std::nullopt)
  {
    return doReadDigitalInputPort(port_number, timeout);
  }

  void writeDigitalOutputPort(uint16_t port_number, bool port_value, std::optional<double> timeout = std::nullopt)
  {
    doWriteDigitalOutputPort(port_number, port_value, timeout);
  }

protected:
  Robot_driver() = default;

  virtual void doConnect(std::optional<double> timeout) = 0;
  virtual void doDisconnect(std::optional<double> timeout) = 0;
  virtual void doInitializeRemoteMotion(std::optional<double> timeout) = 0;
  virtual void doPause(std::optional<double> timeout) = 0;
  virtual void doResume(std::optional<double> timeout) = 0;
  virtual void doAbort(std::optional<double> timeout) = 0;
  virtual void doReset(std::optional<double> timeout) = 0;
  virtual void doSetSpeedOverride(int value, std::optional<double> timeout) = 0;
  virtual RobotStatus doGetStatus(std::optional<double> timeout) = 0;
  virtual bool doReadDigitalInputPort(uint16_t port_number, std::optional<double> timeout) = 0;
  virtual void doWriteDigitalOutputPort(uint16_t port_number, bool port_value, std::optional<double> timeout) = 0;
};
