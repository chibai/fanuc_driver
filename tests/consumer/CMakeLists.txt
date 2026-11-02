cmake_minimum_required(VERSION 3.22)
project(fanuc_driver_consumer LANGUAGES CXX)

find_package(fanuc_driver 3 REQUIRED)
add_executable(installed_driver_example ../../examples/fanuc_driver_example.cpp)
target_link_libraries(installed_driver_example PRIVATE fanuc_driver::fanuc_driver)
