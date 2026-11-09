cmake_minimum_required(VERSION 3.22)
project(fanuc_driver_consumer LANGUAGES CXX)

find_package(fanuc_driver 3 REQUIRED)
add_executable(installed_driver_example ../../examples/connect_fanuc.cpp)
target_link_libraries(installed_driver_example PRIVATE fanuc_driver::fanuc_driver)
add_executable(installed_core core.cpp)
target_link_libraries(installed_core PRIVATE fanuc_driver::fanuc_driver)
