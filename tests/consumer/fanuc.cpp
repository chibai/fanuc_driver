// SPDX-License-Identifier: Apache-2.0

#include "robot_driver/drivers/fanuc/robot.hpp"

#include <type_traits>
#include <utility>

#if __has_include("robot_driver/drivers/fanuc/packets.hpp") || __has_include("robot_driver/drivers/fanuc/types.hpp")
#error "RMI protocol headers must not be installed"
#endif

template <typename T>
concept HasControllerError = requires(T status) { status.controller_error; };

static_assert(!HasControllerError<ResponseStatus>);
static_assert(std::is_same_v<decltype(std::declval<Fanuc_driver&>().connect()), ResponseStatus>);
static_assert(std::is_same_v<decltype(std::declval<Fanuc_driver&>().disconnect()), ResponseStatus>);
static_assert(std::is_same_v<decltype(std::declval<Fanuc_driver&>().initializeRemoteMotion()), ResponseStatus>);
static_assert(std::is_same_v<decltype(std::declval<Fanuc_driver&>().pause()), ResponseStatus>);
static_assert(std::is_same_v<decltype(std::declval<Fanuc_driver&>().resume()), ResponseStatus>);
static_assert(std::is_same_v<decltype(std::declval<Fanuc_driver&>().abort()), ResponseStatus>);
static_assert(std::is_same_v<decltype(std::declval<Fanuc_driver&>().reset()), ResponseStatus>);
static_assert(std::is_same_v<decltype(std::declval<Fanuc_driver&>().setSpeedOverride(10)), ResponseStatus>);
static_assert(std::is_same_v<decltype(std::declval<Fanuc_driver&>().getStatus()), Response<RobotStatus>>);
static_assert(std::is_same_v<decltype(std::declval<Fanuc_driver&>().readDigitalInputPort(1)),
                             std::expected<bool, ResponseStatus>>);
static_assert(std::is_same_v<decltype(std::declval<Fanuc_driver&>().writeDigitalOutputPort(1, false)), ResponseStatus>);

int main()
{
  return ResponseStatus{}.code();
}
