# SPDX-FileCopyrightText: 2025, FANUC America Corporation
# SPDX-FileCopyrightText: 2025, FANUC CORPORATION
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.22)
project(fanuc_client)

option(BUILD_EXAMPLES "Build examples" OFF)

# Find dependencies
find_package(Eigen3 REQUIRED)

add_library(fanuc_client SHARED src/rmi_singleton.cpp)
target_link_libraries(fanuc_client PUBLIC fanuc_driver)
target_include_directories(fanuc_client PUBLIC
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

if (BUILD_TESTING)
  add_subdirectory(test)
endif ()
