cmake_minimum_required(VERSION 3.14)
project(fake_test_node)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(fake_test_node SHARED src/fake_test_node.cpp)



ament_auto_package()
