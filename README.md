# fanuc_driver

A C++23 robot driver library for FANUC controllers.

## Layout

```
CMakeLists.txt     top-level build, install and package export
cmake/             package config template
src/
  rmi/             Remote Motion Interface (JSON over TCP) client
  stream_motion/   Stream Motion (UDP) client
  fanuc_client/    High-level client combining RMI and Stream Motion
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
target_link_libraries(my_app PRIVATE fanuc_driver::fanuc_client)
```

Requires Eigen3 and a C++23 compiler. See [docs/stream_motion.md](docs/stream_motion.md).
