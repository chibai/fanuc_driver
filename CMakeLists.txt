# SPDX-FileCopyrightText: 2025, FANUC America Corporation
# SPDX-FileCopyrightText: 2025, FANUC CORPORATION
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.22)
project(fanuc_driver VERSION 3.0.0 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
include(GNUInstallDirs)
option(BUILD_TESTING "Build tests" OFF)
option(BUILD_EXAMPLES "Build examples" OFF)

include(FetchContent)
find_package(Boost 1.74 REQUIRED)
find_package(nlohmann_json 3.11 REQUIRED)

find_package(Threads REQUIRED)
find_package(Iconv REQUIRED)

add_library(fanuc_driver SHARED
    src/core/error.cpp
    src/drivers/fanuc/robot.cpp
    src/drivers/fanuc/serialization.cpp
    src/drivers/fanuc/deserialization.cpp
)
add_library(fanuc_driver::fanuc_driver ALIAS fanuc_driver)
target_compile_features(fanuc_driver PUBLIC cxx_std_23)
set_target_properties(fanuc_driver PROPERTIES CXX_EXTENSIONS OFF)
target_include_directories(fanuc_driver PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(fanuc_driver PRIVATE nlohmann_json::nlohmann_json Boost::headers Threads::Threads Iconv::Iconv)

if(BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

if(BUILD_TESTING)
  FetchContent_Declare(
      googletest
      URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
  )
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  set(INSTALL_GTEST OFF CACHE BOOL "Enable installation of googletest" FORCE)
  set(BUILD_GMOCK OFF CACHE BOOL "Build Google Mock" FORCE)
  FetchContent_MakeAvailable(googletest)
  enable_testing()
  add_subdirectory(tests)
endif()

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/fanuc_driverConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)
configure_package_config_file(cmake/fanuc_driverConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/fanuc_driverConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/fanuc_driver
)
install(TARGETS fanuc_driver
    EXPORT fanuc_driverExports
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(EXPORT fanuc_driverExports
    FILE fanuc_driverTargets.cmake
    NAMESPACE fanuc_driver::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/fanuc_driver
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/fanuc_driverConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/fanuc_driverConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/fanuc_driver
)
