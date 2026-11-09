// SPDX-License-Identifier: Apache-2.0

#include "robot_driver/drivers/fanuc/robot.hpp"

#include "detail/rmi_driver.hpp"

struct Fanuc_driver::Impl
{
  explicit Impl(const std::string& address, uint16_t port) : driver(address, port)
  {
  }

  Robot_driver& core()
  {
    return driver;
  }

  FanucRmiDriver driver;
};

Fanuc_driver::Fanuc_driver(const std::string& robot_ip_address, uint16_t port)
  : impl_(std::make_unique<Impl>(robot_ip_address, port))
{
}

Fanuc_driver::~Fanuc_driver() = default;

ResponseStatus Fanuc_driver::doConnect(std::optional<double> timeout)
{
  return impl_->core().connect(timeout);
}

ResponseStatus Fanuc_driver::doDisconnect(std::optional<double> timeout)
{
  return impl_->core().disconnect(timeout);
}

ResponseStatus Fanuc_driver::doInitializeRemoteMotion(std::optional<double> timeout)
{
  return impl_->core().initializeRemoteMotion(timeout);
}

ResponseStatus Fanuc_driver::doPause(std::optional<double> timeout)
{
  return impl_->core().pause(timeout);
}

ResponseStatus Fanuc_driver::doResume(std::optional<double> timeout)
{
  return impl_->core().resume(timeout);
}

ResponseStatus Fanuc_driver::doAbort(std::optional<double> timeout)
{
  return impl_->core().abort(timeout);
}

ResponseStatus Fanuc_driver::doReset(std::optional<double> timeout)
{
  return impl_->core().reset(timeout);
}

ResponseStatus Fanuc_driver::doSetSpeedOverride(int value, std::optional<double> timeout)
{
  return impl_->core().setSpeedOverride(value, timeout);
}

Response<RobotStatus> Fanuc_driver::doGetStatus(std::optional<double> timeout)
{
  return impl_->core().getStatus(timeout);
}

std::expected<bool, ResponseStatus> Fanuc_driver::doReadDigitalInputPort(uint16_t port_number,
                                                                         std::optional<double> timeout)
{
  return impl_->core().readDigitalInputPort(port_number, timeout);
}

ResponseStatus Fanuc_driver::doWriteDigitalOutputPort(uint16_t port_number, bool port_value,
                                                      std::optional<double> timeout)
{
  return impl_->core().writeDigitalOutputPort(port_number, port_value, timeout);
}
