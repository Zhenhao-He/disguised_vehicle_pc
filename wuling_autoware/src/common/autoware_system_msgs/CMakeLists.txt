cmake_minimum_required(VERSION 3.5)
project(autoware_system_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AutowareState.msg"
  "msg/HazardStatus.msg"
  "msg/HazardStatusStamped.msg"
  DEPENDENCIES
    geometry_msgs
    std_msgs
    unique_identifier_msgs
    nav_msgs
    builtin_interfaces
    diagnostic_msgs
  ADD_LINTER_TESTS
)



ament_auto_package()
