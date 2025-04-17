cmake_minimum_required(VERSION 3.14)
project(autoware_trajectory_follower_node)

find_package(autoware_cmake REQUIRED)
autoware_package()

set(CONTROLLER_NODE controller_node)
ament_auto_add_library(${CONTROLLER_NODE} SHARED
  include/autoware/trajectory_follower_node/controller_node.hpp
  src/controller_node.cpp
)

rclcpp_components_register_node(${CONTROLLER_NODE}
  PLUGIN "autoware::motion::control::trajectory_follower_node::Controller"
  EXECUTABLE ${CONTROLLER_NODE}_exe
)

# simple trajectory follower
# set(SIMPLE_TRAJECTORY_FOLLOWER_NODE simple_trajectory_follower)
# ament_auto_add_library(${SIMPLE_TRAJECTORY_FOLLOWER_NODE} SHARED
#   include/autoware/trajectory_follower_node/simple_trajectory_follower.hpp
#   src/simple_trajectory_follower.cpp
# )

# rclcpp_components_register_node(${SIMPLE_TRAJECTORY_FOLLOWER_NODE}
#   PLUGIN "simple_trajectory_follower::SimpleTrajectoryFollower"
#   EXECUTABLE ${SIMPLE_TRAJECTORY_FOLLOWER_NODE}_exe
# )


ament_auto_package(
  INSTALL_TO_SHARE
  param
  launch
  config
)
