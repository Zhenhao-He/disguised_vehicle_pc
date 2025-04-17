cmake_minimum_required(VERSION 3.5)
project(wgs_to_utm_mgrs)


find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(lanelet2_projection REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(msg_interfaces REQUIRED)

add_executable(${PROJECT_NAME}_node 
src/wgs_convertor_node.cpp
src/wgs_convertor.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node 
rclcpp 
std_msgs 
sensor_msgs 
geometry_msgs 
nav_msgs
lanelet2_projection
tf2
tf2_geometry_msgs
msg_interfaces
)

target_link_libraries(${PROJECT_NAME}_node
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${lanelet2_projection_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
)

include_directories(
    include
    ${rclcpp_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${lanelet2_projection_INCLUDE_DIRS}
    ${tf2_INCLUDE_DIRS}
    ${tf2_geometry_msgs_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)


install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

