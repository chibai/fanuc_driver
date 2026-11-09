# SPDX-License-Identifier: Apache-2.0

include(GoogleTest)

add_executable(fanuc_driver_test
    core/error_test.cpp
    core/robot_test.cpp
    drivers/fanuc/robot_test.cpp
    drivers/fanuc/response_test.cpp
    drivers/fanuc/serialization_test.cpp
)
set_target_properties(fanuc_driver_test PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
target_include_directories(fanuc_driver_test PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(fanuc_driver_test PRIVATE
    fanuc_driver::fanuc_driver GTest::gtest_main Boost::headers Threads::Threads
)
gtest_discover_tests(fanuc_driver_test)
