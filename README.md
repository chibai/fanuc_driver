# fanuc_driver

A C++23 robot driver library for FANUC controllers.

## Layout

```
CMakeLists.txt     top-level build, install and package export
cmake/             package config template
src/
  rmi/             Remote Motion Interface (JSON over TCP) client
  stream_motion/   Stream Motion (UDP) client
  fanuc_driver/    Unified fanuc_driver class owning connections and GPIO configuration
  fanuc_client/    GPIO buffers and legacy compatibility entry points
  gpio_config/     YAML GPIO configuration loader
examples/          Example programs
third_party/       Vendored dependencies (git submodules) and patches
docs/              Additional documentation
```

## Build

```bash
git submodule update --init --recursive
cmake -B build -S . -DBUILD_TESTING=ON -DBUILD_EXAMPLES=ON
cmake --build build -j
ctest --test-dir build
```

## Use

```cmake
find_package(fanuc_driver REQUIRED)
target_link_libraries(my_app PRIVATE fanuc_driver::fanuc_driver)
```

The exported target propagates C++23 and Eigen3 requirements to consumers.

```cpp
#include <fanuc_driver/fanuc_driver.hpp>

int main()
{
  fanuc_driver driver("192.168.1.100");
  const auto status = driver.getStatus();
  const auto joints = driver.readJointAnglesRMI().eval();
  driver.writeDigitalOutputPort(1, true);
}
```

Constructing the driver connects to RMI, reads controller capabilities and
fetches motion limits. Network/protocol failures throw exceptions. Destruction
stops and joins the stream worker, stops motion and disconnects RMI without
throwing; cleanup failures are logged. Each driver owns its connections, so
different controllers do not share process-wide state. Drivers cannot be copied
or moved because their workers refer to the owning object. Transport interfaces
can still be injected through the constructor for testing.

## Unified interface

The class exposes RMI status, errors, registers, variables, digital/analog IO,
frames, payloads, program calls and motion instructions as methods. Use
`startRMI()` before issuing RMI motion, or `startRealtimeStream()` for streaming.
The latter starts `STREAM_MOTN` automatically when motion control is enabled;
the controller must have the corresponding TP program installed. Raw RMI motion
and program calls are rejected while streaming; non-motion RMI IO and diagnostics
remain available. RMI and Stream Motion packet formats are unchanged.

GPIO can be configured programmatically with `fanuc_driver::GPIOConfig` or from
YAML using `loadGPIOConfig()`. For example, with [examples/gpio.yaml](examples/gpio.yaml):

```cpp
fanuc_driver driver("192.168.1.100");
driver.loadGPIOConfig("examples/gpio.yaml");
driver.setDoMotnCtrl(false);  // IO-only streaming; do not start motion control.
driver.startRealtimeStream();

using Command = fanuc_driver::GPIOBuffer::CommandGPIOTypes;
using State = fanuc_driver::GPIOBuffer::StatusGPIOTypes;
driver.writeGPIO<Command::DO, bool>(1, true);
if (!driver.sendIOCommand())
{
  throw std::runtime_error("Unable to enqueue GPIO command");
}
// Status is asynchronous; an immediate read may precede the output update.
const bool output = driver.readGPIO<State::DO, bool>(1);
driver.stopRealtimeStream();
```

GPIO ranges use 1-based controller indices. Configuration is validated before
being committed; unsupported, overlapping, zero-length or oversized ranges
throw. Configure GPIO only while stopped. Configuration survives stream restarts;
`configureGPIO(fanuc_driver::GPIOConfig{})` explicitly clears it.
The typed `writeGPIO` methods update the command buffer; `sendIOCommand()` queues
it for transmission. `readGPIO`, `readRobotStatus` and `readForceSensor` refresh
cached state from the receive queue. Joint-angle reads return views into driver
storage; copy them (or use Eigen's `.eval()`) to retain a snapshot.

Call the control/configuration/read/write API from one application thread; the
internal worker exchanges commands and status through SPSC queues. Atomic
stream/motion flags do not make the entire API concurrently callable.
`stopRealtimeStream()` joins the worker and stops the controller; it is safe to
call again after a successful stop. `stopStreaming()` only requests that the
worker exit; follow it with `stopRealtimeStream()` for controller shutdown.
Worker communication failures stop streaming and are logged, not allowed to
terminate the process. Use `checkStreamError()` after a streaming loop to rethrow
the original failure; state reads and `stopRealtimeStream()` also report errors.
Restarting the stream clears old queues and worker errors.

The motion example is [examples/fanuc_driver_example.cpp](examples/fanuc_driver_example.cpp)
(`build/examples/fanuc_driver_example`). It **moves the robot**; review it and
ensure the controller is prepared before running it.

## Compatibility

Existing source using `<fanuc_client/fanuc_client.hpp>` and
`fanuc_client::FanucClient` continues to work through a type alias; the
`fanuc_driver::fanuc_client` CMake target is retained. Rebuild consumers after
upgrading; binary compatibility is not maintained.
The global `FanucClient::get_instance()` entry point was removed: pass driver
references explicitly. `RMISingleton` remains a legacy standalone utility, but
constructing a driver no longer registers a connection with it.
Lower-level RMI, Stream Motion and GPIO configuration targets remain available.

Requires Eigen3 and a C++23 compiler. See [docs/stream_motion.md](docs/stream_motion.md)
for the low-level UDP interface.
