#include <gtest/gtest.h>

#include <memory>
#include <type_traits>
#include <vector>

#include "robot_driver/core/robot.hpp"

namespace
{
class TestDriver final : public Robot_driver
{
public:
  explicit TestDriver(bool& destroyed) : destroyed_(destroyed)
  {
  }

  ~TestDriver() override
  {
    destroyed_ = true;
  }

  std::vector<std::optional<double>> timeouts;
  std::vector<int> operations;
  int speed = 0;
  uint16_t input_port = 0;
  uint16_t output_port = 0;
  bool output_value = false;

private:
  void record(int operation, std::optional<double> timeout)
  {
    operations.push_back(operation);
    timeouts.push_back(timeout);
  }

  ResponseStatus doConnect(std::optional<double> timeout) override
  {
    record(0, timeout);
    return {};
  }

  ResponseStatus doDisconnect(std::optional<double> timeout) override
  {
    record(1, timeout);
    return {};
  }

  ResponseStatus doInitializeRemoteMotion(std::optional<double> timeout) override
  {
    record(2, timeout);
    return {};
  }

  ResponseStatus doPause(std::optional<double> timeout) override
  {
    record(3, timeout);
    return {};
  }

  ResponseStatus doResume(std::optional<double> timeout) override
  {
    record(4, timeout);
    return {};
  }

  ResponseStatus doAbort(std::optional<double> timeout) override
  {
    record(5, timeout);
    return {};
  }

  ResponseStatus doReset(std::optional<double> timeout) override
  {
    record(6, timeout);
    return {};
  }

  ResponseStatus doSetSpeedOverride(int value, std::optional<double> timeout) override
  {
    record(7, timeout);
    speed = value;
    return {};
  }

  Response<RobotStatus> doGetStatus(std::optional<double> timeout) override
  {
    record(8, timeout);
    return { {}, RobotStatus{ false, speed } };
  }

  std::expected<bool, ResponseStatus> doReadDigitalInputPort(uint16_t port_number,
                                                             std::optional<double> timeout) override
  {
    record(9, timeout);
    input_port = port_number;
    return true;
  }

  ResponseStatus doWriteDigitalOutputPort(uint16_t port_number, bool port_value, std::optional<double> timeout) override
  {
    record(10, timeout);
    output_port = port_number;
    output_value = port_value;
    return {};
  }

  bool& destroyed_;
};

static_assert(std::is_abstract_v<Robot_driver>);
static_assert(std::has_virtual_destructor_v<Robot_driver>);
static_assert(!std::is_copy_constructible_v<Robot_driver>);
static_assert(!std::is_move_constructible_v<TestDriver>);
}  // namespace

TEST(RobotDriver, DispatchesWithoutBrandSpecificTypes)
{
  bool destroyed = false;
  auto backend = std::make_unique<TestDriver>(destroyed);
  const auto* recorded = backend.get();
  std::unique_ptr<Robot_driver> driver = std::move(backend);
  EXPECT_TRUE(driver->connect().ok());
  EXPECT_TRUE(driver->disconnect(2.0).ok());
  EXPECT_TRUE(driver->initializeRemoteMotion(-1.0).ok());
  EXPECT_TRUE(driver->pause(0.5).ok());
  EXPECT_TRUE(driver->resume(0.0).ok());
  EXPECT_TRUE(driver->abort().ok());
  EXPECT_TRUE(driver->reset(3.0).ok());
  EXPECT_TRUE(driver->setSpeedOverride(25, 4.0).ok());
  const auto status = driver->getStatus(5.0);
  ASSERT_TRUE(status.ok());
  EXPECT_FALSE(status.data->servo_ready);
  EXPECT_EQ(status.data->speed_override, 25);
  const auto input = driver->readDigitalInputPort(17, 6.0);
  ASSERT_TRUE(input.has_value());
  EXPECT_TRUE(*input);
  EXPECT_TRUE(driver->writeDigitalOutputPort(19, true, 7.0).ok());
  EXPECT_EQ(recorded->speed, 25);
  EXPECT_EQ(recorded->input_port, 17);
  EXPECT_EQ(recorded->output_port, 19);
  EXPECT_TRUE(recorded->output_value);
  EXPECT_EQ(recorded->operations, (std::vector<int>{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 }));
  EXPECT_EQ(recorded->timeouts, (std::vector<std::optional<double>>{ std::nullopt, 2.0, -1.0, 0.5, 0.0, std::nullopt,
                                                                     3.0, 4.0, 5.0, 6.0, 7.0 }));
  EXPECT_FALSE(destroyed);
  driver.reset();
  EXPECT_TRUE(destroyed);
}
