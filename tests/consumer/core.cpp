// SPDX-License-Identifier: Apache-2.0

#include <type_traits>

#include <robot_driver/core/error.hpp>
#include <robot_driver/core/robot.hpp>

static_assert(std::is_abstract_v<Robot_driver>);

int main()
{
  return ResponseStatus{}.code();
}
