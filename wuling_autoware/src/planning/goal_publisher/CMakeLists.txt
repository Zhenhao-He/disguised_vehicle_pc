cmake_minimum_required(VERSION 3.8)
project(goal_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(parking_slot_detection_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(msg_interfaces REQUIRED)
find_package(wgs_to_utm_mgrs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(goal_publisher_node src/goal_publisher_node.cpp)
ament_target_dependencies(goal_publisher_node rclcpp geometry_msgs parking_slot_detection_msgs tf2 tf2_geometry_msgs wgs_to_utm_mgrs nav_msgs)


install(TARGETS
  goal_publisher_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
