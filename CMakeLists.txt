# SPDX-FileCopyrightText: 2025, FANUC America Corporation
# SPDX-FileCopyrightText: 2025, FANUC CORPORATION
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.22)
project(fanuc_driver VERSION 2.4.0 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
include(GNUInstallDirs)


# Build the sockpp library
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

FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY ${PROJECT_SOURCE_DIR}/third_party/yaml-cpp
  GIT_TAG 2f86d13775d119edbb69af52e5f566fd65c6953b
)
FetchContent_MakeAvailable(yaml-cpp)
set(YAML_BUILD_SHARED_LIBS OFF CACHE BOOL "Build yaml-cpp shared library")
set(YAML_CPP_INSTALL OFF CACHE BOOL "Enable generation of yaml-cpp install targets")

# Build the reflectcpp library
FetchContent_Declare(
    reflect-cpp
    GIT_REPOSITORY ${PROJECT_SOURCE_DIR}/third_party/reflect-cpp
    GIT_TAG 2a0fdeabe35bdab0e07e05d4f869ca88bd6d2b8c
    PATCH_COMMAND bash ${PROJECT_SOURCE_DIR}/third_party/apply_patch.bash ${PROJECT_SOURCE_DIR}/third_party/reflectcpp.patch
)
set(REFLECTCPP_BUILD_SHARED OFF CACHE BOOL "Build reflectcpp as a shared library")
set(REFLECTCPP_YAML ON CACHE BOOL "Enable YAML support")
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

add_subdirectory(src/stream_motion)
add_subdirectory(src/rmi)
add_subdirectory(src/gpio_config)
add_subdirectory(src/fanuc_driver)
add_subdirectory(src/fanuc_client)

# All driver targets require C++23.
set_target_properties(stream_motion rmi fanuc_client gpio_config fanuc_driver PROPERTIES
    CXX_STANDARD 23
    CXX_STANDARD_REQUIRED ON
)
foreach(target stream_motion rmi fanuc_client gpio_config)
  target_compile_features(${target} PUBLIC cxx_std_23)
  set_target_properties(${target} PROPERTIES CXX_EXTENSIONS OFF)
endforeach()

if (BUILD_EXAMPLES)
  add_subdirectory(examples)
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
include(GNUInstallDirs)
install(TARGETS stream_motion rmi fanuc_client gpio_config fanuc_driver
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
    src/rmi/include/
    src/stream_motion/include/
    src/fanuc_client/include/
    src/gpio_config/include/
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
