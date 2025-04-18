cmake_minimum_required(VERSION 3.5)
project(diagnostic_aggregator)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_C_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/status_item.cpp
  src/analyzer_group.cpp
  src/aggregator.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  "diagnostic_msgs"
  "pluginlib"
  "rclcpp"
  "std_msgs"
)
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "DIAGNOSTIC_AGGREGATOR_BUILDING_DLL")

# see https://github.com/pybind/pybind11/commit/ba33b2fc798418c8c9dfe801c5b9023d3703f417
if(NOT WIN32)
  target_compile_options(${PROJECT_NAME} PRIVATE -Wdeprecated)
endif()

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

set(ANALYZERS "${PROJECT_NAME}_analyzers")
add_library(${ANALYZERS} SHARED
  src/generic_analyzer.cpp
  src/discard_analyzer.cpp
  src/ignore_analyzer.cpp)
target_include_directories(${ANALYZERS} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${ANALYZERS}
  "diagnostic_msgs"
  "pluginlib"
  "rclcpp"
  "std_msgs"
)
target_link_libraries(${ANALYZERS}
  ${PROJECT_NAME})
target_compile_definitions(${ANALYZERS}
  PRIVATE "DIAGNOSTIC_AGGREGATOR_BUILDING_DLL")

# prevent pluginlib from using boost
target_compile_definitions(${ANALYZERS} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

# Aggregator node
add_executable(aggregator_node src/aggregator_node.cpp)
target_link_libraries(aggregator_node
  ${PROJECT_NAME})

# Add analyzer
add_executable(add_analyzer src/add_analyzer.cpp)
ament_target_dependencies(add_analyzer rclcpp rcl_interfaces)



install(
  TARGETS aggregator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS add_analyzer
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME} ${ANALYZERS}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_python_install_package(${PROJECT_NAME})

# Install Example
set(ANALYZER_PARAMS_FILEPATH "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/example_analyzers.yaml")
set(ADD_ANALYZER_PARAMS_FILEPATH "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/example_add_analyzers.yaml")
configure_file(example/example.launch.py.in example.launch.py @ONLY)
install( # launch descriptor
  FILES ${CMAKE_CURRENT_BINARY_DIR}/example.launch.py
  DESTINATION share/${PROJECT_NAME}
)
install( # example publisher
  PROGRAMS example/example_pub.py
  DESTINATION lib/${PROJECT_NAME}
)
install( # example aggregator configration
  FILES example/example_analyzers.yaml example/example_add_analyzers.yaml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(${PROJECT_NAME} plugin_description.xml)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

ament_export_include_directories(include)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(ament_cmake_python)
ament_export_dependencies(diagnostic_msgs)
ament_export_dependencies(pluginlib)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclpy)
ament_export_dependencies(std_msgs)

ament_package()
