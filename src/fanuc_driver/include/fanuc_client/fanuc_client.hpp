// SPDX-FileCopyrightText: 2025-2026, FANUC America Corporation
// SPDX-FileCopyrightText: 2025-2026, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "fanuc_driver/fanuc_driver.hpp"

namespace fanuc_client
{

using FanucClient = ::fanuc_driver;
using GPIOBuffer = ::fanuc_driver::GPIOBuffer;
using RobotStatus = ::fanuc_driver::RobotStatus;
using ForceSensor = ::fanuc_driver::ForceSensor;
using ContactStopMode = ::fanuc_driver::ContactStopMode;

}  // namespace fanuc_client
