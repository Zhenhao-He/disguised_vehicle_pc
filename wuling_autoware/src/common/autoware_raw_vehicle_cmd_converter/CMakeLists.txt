cmake_minimum_required(VERSION 3.14)
project(autoware_raw_vehicle_cmd_converter)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(actuation_map_converter SHARED
  src/accel_map.cpp
  src/brake_map.cpp
  src/steer_map.cpp
  src/csv_loader.cpp
  src/pid.cpp
  src/vgr.cpp
)

ament_auto_add_library(autoware_raw_vehicle_cmd_converter_node_component SHARED
  src/node.cpp
)

target_link_libraries(autoware_raw_vehicle_cmd_converter_node_component actuation_map_converter)

rclcpp_components_register_node(autoware_raw_vehicle_cmd_converter_node_component
  PLUGIN "autoware::raw_vehicle_cmd_converter::RawVehicleCommandConverterNode"
  EXECUTABLE autoware_raw_vehicle_cmd_converter_node
)



ament_auto_package(INSTALL_TO_SHARE
  config
  data
  launch
  test
)

install(PROGRAMS scripts/plot_accel_brake_map.py
  DESTINATION lib/${PROJECT_NAME}
)
