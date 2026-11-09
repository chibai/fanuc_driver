// SPDX-FileCopyrightText: 2025-2026, FANUC America Corporation
// SPDX-FileCopyrightText: 2025-2026, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "robot_driver/core/robot.hpp"

class Fanuc_driver final : public Robot_driver
{
public:
  explicit Fanuc_driver(const std::string& robot_ip_address, uint16_t port = 16001);
  ~Fanuc_driver() override;

  Fanuc_driver(const Fanuc_driver&) = delete;
  Fanuc_driver& operator=(const Fanuc_driver&) = delete;

private:
  ResponseStatus doConnect(std::optional<double> timeout) override;
  ResponseStatus doDisconnect(std::optional<double> timeout) override;
  ResponseStatus doInitializeRemoteMotion(std::optional<double> timeout) override;
  ResponseStatus doPause(std::optional<double> timeout) override;
  ResponseStatus doResume(std::optional<double> timeout) override;
  ResponseStatus doAbort(std::optional<double> timeout) override;
  ResponseStatus doReset(std::optional<double> timeout) override;
  ResponseStatus doSetSpeedOverride(int value, std::optional<double> timeout) override;
  Response<RobotStatus> doGetStatus(std::optional<double> timeout) override;
  std::expected<bool, ResponseStatus> doReadDigitalInputPort(uint16_t port_number,
                                                             std::optional<double> timeout) override;
  ResponseStatus doWriteDigitalOutputPort(uint16_t port_number, bool port_value, std::optional<double> timeout) override;

  struct Impl;
  std::unique_ptr<Impl> impl_;
};
