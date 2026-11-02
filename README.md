# fanuc_driver

A C++23 robot driver library for FANUC controllers.

## Layout

```
CMakeLists.txt     top-level build, install and package export
cmake/             package config template
src/
  fanuc_driver/    Driver API, protocol implementations, buffers and tests
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

The exported target is the single library entry point and propagates C++23 and
Eigen3 requirements to consumers. RMI and Stream Motion implementations are
part of this module; there are no protocol-specific CMake targets or source
modules.

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

Realtime GPIO buffers are optional. If needed, create a
`fanuc_driver::GPIOBuffer::Builder`, add command/status ranges, wrap
`builder.build()` with `std::make_shared<fanuc_driver::GPIOBuffer>(...)`, and
pass it to `startRealtimeStream()`. The separate YAML configuration module has
been removed; RMI I/O methods do not need a realtime GPIO buffer.
`readRobotStatus` and `readForceSensor` refresh cached state from the receive
queue. Joint-angle reads return views into driver storage; copy them (or use
Eigen's `.eval()`) to retain a snapshot.

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

The unified motion example is [examples/fanuc_driver_example.cpp](examples/fanuc_driver_example.cpp)
(`build/fanuc_driver_example`). It **moves the robot**; review it and ensure the
controller is prepared before running it.

## Compatibility

Existing source using `<fanuc_client/fanuc_client.hpp>` and
`fanuc_client::FanucClient` continues to work through a compatibility alias.
Rebuild consumers after upgrading; binary compatibility is not maintained.
The global singleton entry points were removed: create a `fanuc_driver` instance
and pass it explicitly. Link consumers to `fanuc_driver::fanuc_driver`;
protocol detail headers live under `fanuc_driver/detail` and are not separate
modules or build targets.

Requires Eigen3 and a C++23 compiler.
