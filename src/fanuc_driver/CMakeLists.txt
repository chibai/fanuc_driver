# SPDX-FileCopyrightText: 2026, FANUC CORPORATION
#
# SPDX-License-Identifier: Apache-2.0

find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

add_library(fanuc_driver SHARED
    src/fanuc_driver.cpp
    src/gpio_config.cpp
    ../fanuc_client/src/gpio_buffer.cpp
)
add_library(fanuc_driver::fanuc_driver ALIAS fanuc_driver)
target_compile_features(fanuc_driver PUBLIC cxx_std_23)
set_target_properties(fanuc_driver PROPERTIES CXX_EXTENSIONS OFF)
target_link_libraries(fanuc_driver PUBLIC stream_motion rmi gpio_config Eigen3::Eigen
    PRIVATE readerwriterqueue Threads::Threads)
target_include_directories(fanuc_driver PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../fanuc_client/include>
    $<INSTALL_INTERFACE:include>
)
