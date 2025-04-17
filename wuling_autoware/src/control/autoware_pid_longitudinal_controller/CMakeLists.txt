cmake_minimum_required(VERSION 3.14)
project(autoware_pid_longitudinal_controller)

find_package(autoware_cmake REQUIRED)
autoware_package()

set(PID_LON_CON_LIB ${PROJECT_NAME}_lib)
ament_auto_add_library(${PID_LON_CON_LIB} SHARED
  DIRECTORY src
)



ament_auto_package(
  INSTALL_TO_SHARE
  config
)
