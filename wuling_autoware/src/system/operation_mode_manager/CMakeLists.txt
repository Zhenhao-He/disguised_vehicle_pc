cmake_minimum_required(VERSION 3.8)
project(operation_mode_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)

add_executable(operation_mode_manager_node src/operation_mode_manager_node.cpp)
ament_target_dependencies(operation_mode_manager_node rclcpp autoware_adapi_v1_msgs)


install(TARGETS
  operation_mode_manager_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
