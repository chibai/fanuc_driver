// SPDX-FileCopyrightText: 2026, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#include "fanuc_driver/fanuc_driver.hpp"

#include <stdexcept>

namespace
{
using Buffer = fanuc_driver::GPIOBuffer;
using Command = Buffer::CommandGPIOTypes;
using State = Buffer::StatusGPIOTypes;

void AddCommand(Buffer::Builder& builder, const gpio_config::BoolIOCmdConfig& config)
{
  switch (config.type)
  {
    case gpio_config::BoolIOCmdType::DO:
      builder.addCommandConfig<Command::DO, bool>(config.start, config.length);
      return;
    case gpio_config::BoolIOCmdType::RO:
      builder.addCommandConfig<Command::RO, bool>(config.start, config.length);
      return;
    case gpio_config::BoolIOCmdType::F:
      builder.addCommandConfig<Command::F, bool>(config.start, config.length);
      return;
  }
  throw std::invalid_argument("Unsupported GPIO command type.");
}

void AddState(Buffer::Builder& builder, const gpio_config::BoolIOStateConfig& config)
{
  switch (config.type)
  {
    case gpio_config::BoolIOStateType::DI:
      builder.addStatusConfig<State::DI, bool>(config.start, config.length);
      return;
    case gpio_config::BoolIOStateType::DO:
      builder.addStatusConfig<State::DO, bool>(config.start, config.length);
      return;
    case gpio_config::BoolIOStateType::RI:
      builder.addStatusConfig<State::RI, bool>(config.start, config.length);
      return;
    case gpio_config::BoolIOStateType::RO:
      builder.addStatusConfig<State::RO, bool>(config.start, config.length);
      return;
    case gpio_config::BoolIOStateType::F:
      builder.addStatusConfig<State::F, bool>(config.start, config.length);
      return;
  }
  throw std::invalid_argument("Unsupported GPIO state type.");
}
}  // namespace

void fanuc_driver::configureGPIO(const gpio_config::GPIOConfig& config)
{
  if (is_streaming_ || rt_thread_.joinable())
  {
    throw std::logic_error("Stop the real-time stream before configuring GPIO.");
  }

  GPIOBuffer::Builder builder;
  const auto& topic = config.gpio_topic_config;
  if (topic.io_cmd)
  {
    for (const auto& block : *topic.io_cmd)
    {
      AddCommand(builder, block);
    }
  }
  if (topic.io_state)
  {
    for (const auto& block : *topic.io_state)
    {
      AddState(builder, block);
    }
  }
  if (topic.analog_io_cmd)
  {
    for (const auto& block : *topic.analog_io_cmd)
    {
      if (block.type != gpio_config::AnalogIOCmdType::AO)
      {
        throw std::invalid_argument("Unsupported analog GPIO command type.");
      }
      builder.addCommandConfig<Command::AO, uint16_t>(block.start, block.length);
    }
  }
  if (topic.analog_io_state)
  {
    for (const auto& block : *topic.analog_io_state)
    {
      switch (block.type)
      {
        case gpio_config::AnalogIOStateType::AI:
          builder.addStatusConfig<State::AI, uint16_t>(block.start, block.length);
          break;
        case gpio_config::AnalogIOStateType::AO:
          builder.addStatusConfig<State::AO, uint16_t>(block.start, block.length);
          break;
        default:
          throw std::invalid_argument("Unsupported analog GPIO state type.");
      }
    }
  }
  if (topic.num_reg_cmd)
  {
    for (const auto& block : *topic.num_reg_cmd)
    {
      builder.addCommandConfig<Command::FloatReg, float>(block.start, block.length);
    }
  }
  if (topic.num_reg_state)
  {
    for (const auto& block : *topic.num_reg_state)
    {
      builder.addStatusConfig<State::FloatReg, float>(block.start, block.length);
    }
  }

  auto buffer = std::make_shared<GPIOBuffer>(builder.build());
  validateGPIOBuffer(buffer);
  gpio_buffer_ = std::move(buffer);
}

void fanuc_driver::loadGPIOConfig(const std::filesystem::path& path)
{
  configureGPIO(gpio_config::ParseGPIOConfig(path));
}

fanuc_driver::GPIOBuffer& fanuc_driver::gpio_buffer()
{
  if (!gpio_buffer_)
  {
    throw std::logic_error("Configure GPIO before accessing its buffers.");
  }
  return *gpio_buffer_;
}
