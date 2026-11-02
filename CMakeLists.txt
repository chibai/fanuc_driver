# SPDX-FileCopyrightText: 2025, FANUC America Corporation
# SPDX-FileCopyrightText: 2025, FANUC CORPORATION
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.22)
project(fanuc_driver VERSION 2.4.0 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
include(GNUInstallDirs)
option(BUILD_TESTING "Build tests" OFF)
option(BUILD_EXAMPLES "Build examples" OFF)


# Third-party dependencies
include(FetchContent)
FetchContent_Declare(
    sockpp
    GIT_REPOSITORY ${PROJECT_SOURCE_DIR}/third_party/sockpp
    GIT_TAG afdeacba9448c7a77194eed6ab8e1c0b1653c79a
    PATCH_COMMAND bash ${PROJECT_SOURCE_DIR}/third_party/apply_patch.bash ${PROJECT_SOURCE_DIR}/third_party/sockpp.patch
)
set(SOCKPP_BUILD_SHARED OFF CACHE BOOL "Build sockpp as a shared library")
set(SOCKPP_BUILD_STATIC ON CACHE BOOL "Build sockpp as a static library")
FetchContent_MakeAvailable(sockpp)
set_target_properties(sockpp-static PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Create readerwriterqueue target
FetchContent_Declare(
    readerwriterqueue
    GIT_REPOSITORY ${PROJECT_SOURCE_DIR}/third_party/readerwriterqueue
    GIT_TAG 211616e0554f93152ab3108b8d93fbc23174a9d9
    PATCH_COMMAND bash ${PROJECT_SOURCE_DIR}/third_party/apply_patch.bash ${PROJECT_SOURCE_DIR}/third_party/readerwriterqueue.patch
)
FetchContent_MakeAvailable(readerwriterqueue)

# Build the reflectcpp library
FetchContent_Declare(
    reflect-cpp
    GIT_REPOSITORY ${PROJECT_SOURCE_DIR}/third_party/reflect-cpp
    GIT_TAG 2a0fdeabe35bdab0e07e05d4f869ca88bd6d2b8c
    PATCH_COMMAND bash ${PROJECT_SOURCE_DIR}/third_party/apply_patch.bash ${PROJECT_SOURCE_DIR}/third_party/reflectcpp.patch
)
set(REFLECTCPP_BUILD_SHARED OFF CACHE BOOL "Build reflectcpp as a shared library")
set(REFLECTCPP_YAML OFF CACHE BOOL "Disable unused YAML support" FORCE)
FetchContent_MakeAvailable(reflect-cpp)
set_target_properties(reflectcpp PROPERTIES CXX_STANDARD 20)
set_target_properties(reflectcpp PROPERTIES POSITION_INDEPENDENT_CODE ON)

if (BUILD_TESTING)
  include(FetchContent)
  FetchContent_Declare(
      googletest
      URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
  )
  # For Windows: Prevent overriding the parent project's compiler/linker settings
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  set(INSTALL_GTEST OFF CACHE BOOL "Enable installation of googletest." FORCE)
  set(BUILD_GMOCK ON CACHE BOOL "Builds the googlemock subproject." FORCE)
  FetchContent_MakeAvailable(googletest)
  include(GoogleTest)
  enable_testing()
endif ()

find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

add_library(fanuc_driver SHARED
    src/fanuc_driver/src/rmi.cpp
    src/fanuc_driver/src/rmi_serialization.cpp
    src/fanuc_driver/src/rmi_deserialization.cpp
    src/fanuc_driver/src/stream_motion.cpp
    src/fanuc_driver/src/byte_ops.cpp
    src/fanuc_driver/src/fanuc_driver.cpp
    src/fanuc_driver/src/gpio_buffer.cpp
)
add_library(fanuc_driver::fanuc_driver ALIAS fanuc_driver)
target_compile_features(fanuc_driver PUBLIC cxx_std_23)
set_target_properties(fanuc_driver PROPERTIES
    CXX_STANDARD 23
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)
target_include_directories(fanuc_driver PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/fanuc_driver/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(fanuc_driver
    PUBLIC Eigen3::Eigen
    PRIVATE reflectcpp Sockpp::sockpp readerwriterqueue Threads::Threads
)

if (BUILD_EXAMPLES)
  add_executable(fanuc_driver_example examples/fanuc_driver_example.cpp)
  target_link_libraries(fanuc_driver_example PRIVATE fanuc_driver::fanuc_driver)
endif ()

if (BUILD_TESTING)
  add_executable(rmi_test src/fanuc_driver/test/rmi_test.cpp)
  target_link_libraries(rmi_test PRIVATE fanuc_driver::fanuc_driver GTest::gtest_main)
  gtest_discover_tests(rmi_test)

  add_executable(stream_motion_test src/fanuc_driver/test/stream_motion_test.cpp)
  target_link_libraries(stream_motion_test PRIVATE fanuc_driver::fanuc_driver GTest::gtest_main Sockpp::sockpp)
  gtest_discover_tests(stream_motion_test)

  add_executable(fanuc_driver_test src/fanuc_driver/test/fanuc_driver_test.cpp)
  target_link_libraries(fanuc_driver_test PRIVATE fanuc_driver::fanuc_driver GTest::gtest_main GTest::gmock_main)
  gtest_discover_tests(fanuc_driver_test)

  add_executable(gpio_buffer_test src/fanuc_driver/test/gpio_buffer_test.cpp)
  target_link_libraries(gpio_buffer_test PRIVATE fanuc_driver::fanuc_driver GTest::gtest_main GTest::gmock_main)
  gtest_discover_tests(gpio_buffer_test)
endif ()

# Create cmake config and version files
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/fanuc_driverConfigVersion.cmake
    VERSION
    ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
    ARCH_INDEPENDENT
)
configure_package_config_file(cmake/fanuc_driverConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/fanuc_driverConfig.cmake
    INSTALL_DESTINATION
    ${CMAKE_INSTALL_LIBDIR}/cmake/fanuc_driver/
)

# --- Install ---
install(TARGETS fanuc_driver
    EXPORT fanuc_driverExports
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(EXPORT fanuc_driverExports
    FILE
    fanuc_driverTargets.cmake
    NAMESPACE
    fanuc_driver::
    DESTINATION
    ${CMAKE_INSTALL_LIBDIR}/cmake/fanuc_driver
)
install(DIRECTORY
    src/fanuc_driver/include/
    DESTINATION
    ${CMAKE_INSTALL_INCLUDEDIR}
)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/fanuc_driverConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/fanuc_driverConfigVersion.cmake
    DESTINATION
    ${CMAKE_INSTALL_LIBDIR}/cmake/fanuc_driver
)
