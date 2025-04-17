cmake_minimum_required(VERSION 3.5)
project(zlg_msgs)

find_package(ament_cmake_auto REQUIRED)
find_package(std_msgs REQUIRED)  # 添加这一行

ament_auto_find_build_dependencies()

set(msg_files
  "msg/MyMessage.msg"
  "msg/CanFrame.msg"
  "msg/Frame.msg"
  "msg/Frames.msg"
  "msg/CtrlCmd.msg"
  "msg/VehResponse.msg"
  "msg/ApaReport.msg"
  "msg/TrackCmd.msg"
  "msg/ExpectSpeed.msg")

set(msg_dependencies
  std_msgs
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES ${msg_dependencies}
  ADD_LINTER_TESTS)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()

