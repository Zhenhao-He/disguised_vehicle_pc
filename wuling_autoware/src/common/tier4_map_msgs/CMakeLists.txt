cmake_minimum_required(VERSION 3.5)
project(tier4_map_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/MapProjectorInfo.msg
  DEPENDENCIES
    geographic_msgs
)



ament_auto_package()
