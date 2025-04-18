cmake_minimum_required(VERSION 3.16)
project(generate_parameter_library_example)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion -Wold-style-cast)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)

generate_parameter_library(admittance_controller_parameters
  src/parameters.yaml
  include/generate_parameter_library_example/example_validators.hpp
)

add_library(minimal_publisher SHARED
  src/minimal_publisher.cpp
)
target_include_directories(minimal_publisher PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/generate_parameter_library_example>
)
target_link_libraries(minimal_publisher
  PUBLIC
    admittance_controller_parameters
    rclcpp::rclcpp
    rclcpp_components::component
)
rclcpp_components_register_node(minimal_publisher
  PLUGIN "admittance_controller::MinimalPublisher"
  EXECUTABLE test_node
)


install(
  DIRECTORY include/
  DESTINATION include/generate_parameter_library_example
)

install(TARGETS minimal_publisher admittance_controller_parameters
  EXPORT export_generate_parameter_library_example
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS test_node
  DESTINATION lib/generate_parameter_library_example
)

ament_export_targets(export_generate_parameter_library_example HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_components)
ament_package()
