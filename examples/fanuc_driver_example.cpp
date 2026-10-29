// SPDX-FileCopyrightText: 2025, FANUC America Corporation
// SPDX-FileCopyrightText: 2025, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#include <chrono>
#include <cmath>
#include <iostream>
#include <numbers>
#include <stdexcept>

#include <Eigen/Core>

#include "fanuc_driver/fanuc_driver.hpp"

int main()
{
  try
  {
    fanuc_driver driver("192.168.1.100");
    const double v_peak = 1000.0;
    const double payload = 0.0;
    std::vector<double> vel_limit;
    std::vector<double> acc_limit;
    std::vector<double> jerk_limit;
    driver.getLimits(v_peak, payload, vel_limit, acc_limit, jerk_limit);

    // Configure IO
    fanuc_driver::GPIOConfig gpio_config;
    gpio_config.gpio_topic_config.num_reg_cmd = { { .start = 1, .length = 10 } };
    gpio_config.gpio_topic_config.num_reg_state = { { .start = 1, .length = 10 } };
    driver.configureGPIO(gpio_config);
    using Command = fanuc_driver::GPIOBuffer::CommandGPIOTypes;
    using State = fanuc_driver::GPIOBuffer::StatusGPIOTypes;

    driver.startRMI();
    Eigen::VectorXd initial_command = Eigen::VectorXd::Zero(9);
    initial_command[4] = -90;
    driver.writeJointTargetRMI(initial_command);

    driver.startRealtimeStream();
    Eigen::VectorXd current_command = initial_command;
    driver.writeJointTarget(initial_command);

    // Generate a sine wave motion for 3 seconds
    const int period = 2800;
    const auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < std::chrono::milliseconds(1 * period) && driver.isStreaming())
    {
      std::cout << "j1: " << driver.readJointAngles()[0] << " " << std::endl;
      const double time = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
      const double omega = 2 * std::numbers::pi / (static_cast<double>(period) / 1000);
      current_command.array() = initial_command.array() + 20.0 * (1 + -cos(time * omega));
      driver.writeJointTarget(current_command);

      const auto value = static_cast<float>(time);
      for (int i = 1; i <= 10; ++i)
      {
        driver.writeGPIO<Command::FloatReg, float>(i, value + i - 1);
      }
      if (!driver.sendIOCommand())
      {
        throw std::runtime_error("Unable to enqueue GPIO command.");
      }

      for (int i = 1; i <= 10; ++i)
      {
        std::cout << "reg value " << i << ": " << driver.readGPIO<State::FloatReg, float>(i) << std::endl;
      }

      std::this_thread::sleep_for(std::chrono::microseconds(10 * driver.getControlPeriod()));
    }
    driver.checkStreamError();
    driver.stopRealtimeStream();
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << std::endl;
    return 1;
  }
}
