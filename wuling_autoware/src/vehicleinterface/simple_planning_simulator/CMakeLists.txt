cmake_minimum_required(VERSION 3.14)
project(simple_planning_simulator)

find_package(autoware_cmake REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development)
find_package(learning_based_vehicle_model REQUIRED)
autoware_package()

# Component
ament_auto_add_library(${PROJECT_NAME} SHARED
  include/simple_planning_simulator/simple_planning_simulator_core.hpp
  include/simple_planning_simulator/visibility_control.hpp
  src/simple_planning_simulator/simple_planning_simulator_core.cpp
  src/simple_planning_simulator/vehicle_model/sim_model_interface.cpp
  src/simple_planning_simulator/vehicle_model/sim_model_ideal_steer_vel.cpp
  src/simple_planning_simulator/vehicle_model/sim_model_ideal_steer_acc.cpp
  src/simple_planning_simulator/vehicle_model/sim_model_ideal_steer_acc_geared.cpp
  src/simple_planning_simulator/vehicle_model/sim_model_delay_steer_vel.cpp
  src/simple_planning_simulator/vehicle_model/sim_model_learned_steer_vel.cpp
  src/simple_planning_simulator/vehicle_model/sim_model_delay_steer_acc.cpp
  src/simple_planning_simulator/vehicle_model/sim_model_delay_steer_acc_geared.cpp
  src/simple_planning_simulator/vehicle_model/sim_model_delay_steer_acc_geared_wo_fall_guard.cpp
  src/simple_planning_simulator/vehicle_model/sim_model_delay_steer_map_acc_geared.cpp
  src/simple_planning_simulator/vehicle_model/sim_model_actuation_cmd.cpp
  src/simple_planning_simulator/utils/csv_loader.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC ${Python3_INCLUDE_DIRS} ${learning_based_vehicle_model_INCLUDE_DIRS})
# Node executable
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "simulation::simple_planning_simulator::SimplePlanningSimulator"
  EXECUTABLE ${PROJECT_NAME}_exe
)



ament_auto_package(INSTALL_TO_SHARE param data launch test)
