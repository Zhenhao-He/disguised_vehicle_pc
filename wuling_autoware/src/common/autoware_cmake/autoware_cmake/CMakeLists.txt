cmake_minimum_required(VERSION 3.14)

project(autoware_cmake NONE)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS
  "autoware_cmake-extras.cmake"
)



ament_auto_package(
  INSTALL_TO_SHARE
    cmake
)
