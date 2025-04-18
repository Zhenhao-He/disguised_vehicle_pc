cmake_minimum_required(VERSION 3.5)
project(tier4_control_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GateMode.msg"
  "msg/ExternalCommandSelectorMode.msg"
  "srv/ExternalCommandSelect.srv"
  "msg/IsStartRequested.msg"
  "msg/IsPaused.msg"
  "srv/SetPause.srv"
  "msg/IsStopped.msg"
  "srv/SetStop.srv"
  DEPENDENCIES
    autoware_common_msgs
    std_msgs
)

ament_auto_package()
