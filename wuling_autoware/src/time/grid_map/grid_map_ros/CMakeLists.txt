cmake_minimum_required(VERSION 3.5)
project(grid_map_ros)

## Find catkin macros and libraries
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grid_map_cmake_helpers REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_cv REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)

grid_map_package()

set(dependencies
  cv_bridge
  geometry_msgs
  grid_map_core
  grid_map_cv
  grid_map_msgs
  nav2_msgs
  nav_msgs
  rcutils
  rosbag2_cpp
  sensor_msgs
  std_msgs
  tf2
  visualization_msgs
  rclcpp
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
)

## Declare a cpp library
add_library(${PROJECT_NAME} SHARED
  src/GridMapRosConverter.cpp
  src/GridMapMsgHelpers.cpp
  src/PolygonRosConverter.cpp
)

ament_target_dependencies(${PROJECT_NAME} SYSTEM
  ${dependencies}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

#############
## Testing ##
#############


if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_export_libraries(${PROJECT_NAME})
ament_package()
