// SPDX-FileCopyrightText: 2025-2026, FANUC America Corporation
// SPDX-FileCopyrightText: 2025-2026, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <mutex>

#include "fanuc_driver/fanuc_driver.hpp"

namespace fanuc_client
{

using FanucClient = ::fanuc_driver;

// Legacy utility; driver instances never use this shared connection.
class RMISingleton
{
public:
  static std::shared_ptr<rmi::RMIConnectionInterface> creatNewRMIInstance(const std::string& robot_ip_address,
                                                                          uint16_t rmi_port = 16001);
  static std::shared_ptr<rmi::RMIConnectionInterface> getRMIInstance();
  static std::shared_ptr<rmi::RMIConnectionInterface>
  setRMIInstance(std::unique_ptr<rmi::RMIConnectionInterface> rmi_connection);

private:
  RMISingleton();
  ~RMISingleton();
  RMISingleton(const RMISingleton&) = delete;
  RMISingleton& operator=(const RMISingleton&) = delete;

  static RMISingleton& getInstance();
  std::mutex mtx_;
  std::shared_ptr<rmi::RMIConnectionInterface> rmi_connection_interface_;
};

}  // namespace fanuc_client
