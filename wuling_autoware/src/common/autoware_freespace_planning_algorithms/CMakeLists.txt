cmake_minimum_required(VERSION 3.14)
project(autoware_freespace_planning_algorithms)

find_package(autoware_cmake REQUIRED)
find_package(python_cmake_module REQUIRED)

autoware_package()
ament_python_install_package(${PROJECT_NAME})

ament_auto_add_library(reeds_shepp SHARED
  src/reeds_shepp.cpp
)

ament_auto_add_library(rrtstar_core SHARED
  src/rrtstar_core.cpp
)

target_link_libraries(rrtstar_core
  reeds_shepp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/abstract_algorithm.cpp
  src/astar_search.cpp
  src/rrtstar.cpp
)

target_link_libraries(${PROJECT_NAME}
  reeds_shepp
)



ament_auto_package(
  INSTALL_TO_SHARE
)

install(PROGRAMS
  test/debug_plot.py
  DESTINATION lib/${PROJECT_NAME}
)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)

pybind11_add_module(${PROJECT_NAME}_pybind SHARED
  scripts/bind/astar_search_pybind.cpp
)

include_directories(${PROJECT_NAME}_pybind PRIVATE
  include/
)
target_link_libraries(${PROJECT_NAME}_pybind PRIVATE
  ${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}_pybind
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)
