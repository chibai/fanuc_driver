# SPDX-License-Identifier: Apache-2.0

add_executable(fanuc_driver_example connect_fanuc.cpp)
set_target_properties(fanuc_driver_example PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
target_link_libraries(fanuc_driver_example PRIVATE fanuc_driver::fanuc_driver)
