// Copyright 2021 The Autoware Foundation.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "simple_planning_simulator/simple_planning_simulator_core.hpp"

#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/universe_utils/geometry/geometry.hpp"
#include "autoware/universe_utils/ros/msg_covariance.hpp"
#include "autoware/universe_utils/ros/update_param.hpp"
#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "simple_planning_simulator/vehicle_model/sim_model.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

using namespace std::literals::chrono_literals;

namespace
{
autoware_vehicle_msgs::msg::VelocityReport to_velocity_report(
  const std::shared_ptr<SimModelInterface> vehicle_model_ptr)
{
  autoware_vehicle_msgs::msg::VelocityReport velocity;
  velocity.longitudinal_velocity = static_cast<double>(vehicle_model_ptr->getVx());
  velocity.lateral_velocity = 0.0F;
  velocity.heading_rate = static_cast<double>(vehicle_model_ptr->getWz());
  return velocity;
}

nav_msgs::msg::Odometry to_odometry(
  const std::shared_ptr<SimModelInterface> vehicle_model_ptr, const double ego_pitch_angle)
{
  nav_msgs::msg::Odometry odometry;
  odometry.pose.pose.position.x = vehicle_model_ptr->getX();
  odometry.pose.pose.position.y = vehicle_model_ptr->getY();
  odometry.pose.pose.position.z = 0;
  odometry.pose.pose.orientation = autoware::universe_utils::createQuaternionFromRPY(
    0.0, ego_pitch_angle, vehicle_model_ptr->getYaw());
  odometry.twist.twist.linear.x = vehicle_model_ptr->getVx();
  odometry.twist.twist.angular.z = vehicle_model_ptr->getWz();

  return odometry;
}

autoware_vehicle_msgs::msg::SteeringReport to_steering_report(
  const std::shared_ptr<SimModelInterface> vehicle_model_ptr)
{
  autoware_vehicle_msgs::msg::SteeringReport steer;
  steer.steering_tire_angle = static_cast<double>(vehicle_model_ptr->getSteer());
  return steer;
}

}  // namespace

namespace simulation
{
namespace simple_planning_simulator
{

SimplePlanningSimulator::SimplePlanningSimulator(const rclcpp::NodeOptions & options)
: Node("simple_planning_simulator", options), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
{
  using rclcpp::QoS;
  using std::placeholders::_1;
  using std::placeholders::_2;

  simulated_frame_id_ = declare_parameter("simulated_frame_id", "base_link");
  origin_frame_id_ = declare_parameter("origin_frame_id", "odom");
  add_measurement_noise_ = declare_parameter("add_measurement_noise", false);
  enable_road_slope_simulation_ = declare_parameter("enable_road_slope_simulation", false);
  
  current_input_command_ = Control();
  sub_ackermann_cmd_ = create_subscription<Control>("input/ackermann_control_command", QoS{1},
    [this](const Control::ConstSharedPtr msg) { current_input_command_ = *msg; });
  sub_gear_cmd_ = create_subscription<GearCommand>(
    "input/gear_command", QoS{1},
    [this](const GearCommand::ConstSharedPtr msg) { current_gear_cmd_ = *msg; }); 
  sub_turn_indicators_cmd_ = create_subscription<TurnIndicatorsCommand>(
    "input/turn_indicators_command", QoS{1},
    std::bind(&SimplePlanningSimulator::on_turn_indicators_cmd, this, _1));
  sub_hazard_lights_cmd_ = create_subscription<HazardLightsCommand>(
    "input/hazard_lights_command", QoS{1},
    std::bind(&SimplePlanningSimulator::on_hazard_lights_cmd, this, _1));
    
  pub_odom_                   = create_publisher<Odometry>("output/odometry", QoS{1});
  pub_current_pose_           = create_publisher<PoseWithCovarianceStamped>("output/pose", QoS{1});
  pub_velocity_               = create_publisher<VelocityReport>("output/twist", QoS{1});
  pub_acc_                    = create_publisher<AccelWithCovarianceStamped>("output/acceleration", QoS{1});
  pub_imu_                    = create_publisher<Imu>("output/imu", QoS{1});
  pub_steer_                  = create_publisher<SteeringReport>("output/steering", QoS{1});
  pub_tf_                     = create_publisher<tf2_msgs::msg::TFMessage>("output/tf", QoS{1});
  pub_actuation_status_       = create_publisher<ActuationStatusStamped>("output/actuation_status", QoS{1});
  
  pub_control_mode_report_    = create_publisher<ControlModeReport>("output/control_mode_report", QoS{1});
  pub_gear_report_            = create_publisher<GearReport>("output/gear_report", QoS{1});
  pub_turn_indicators_report_ = create_publisher<TurnIndicatorsReport>("output/turn_indicators_report", QoS{1});
  pub_hazard_lights_report_   = create_publisher<HazardLightsReport>("output/hazard_lights_report", QoS{1});

  timer_sampling_time_ms_ = static_cast<uint32_t>(declare_parameter("timer_sampling_time_ms", 25));
  on_timer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::milliseconds(timer_sampling_time_ms_),
    std::bind(&SimplePlanningSimulator::on_timer, this));
 
  const auto vehicle_model_type_str = declare_parameter("vehicle_model_type", "IDEAL_STEER_VEL");
  initialize_vehicle_model(vehicle_model_type_str);
 
  //初始化为世界坐标系原点
  {
    current_odometry_.pose.pose.position.x =0.0;
    current_odometry_.pose.pose.position.y =0.0;
    current_odometry_.pose.pose.position.z =0.0;
    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, 0);  // 输入欧拉角：roll, pitch, yaw (单位：弧度)
    current_odometry_.pose.pose.orientation.x = quaternion.x();
    current_odometry_.pose.pose.orientation.y = quaternion.y();
    current_odometry_.pose.pose.orientation.z = quaternion.z();
    current_odometry_.pose.pose.orientation.w = quaternion.w();
    
    //set_map_baselink_transforms(current_odometry_);
    set_initial_state(current_odometry_.pose.pose, Twist{});  // initialize with 0 for all variables

  } 
 
  // measurement noise
  {
    std::random_device seed;
    auto & m = measurement_noise_;
    m.rand_engine_ = std::make_shared<std::mt19937>(seed());
    double pos_noise_stddev = declare_parameter("pos_noise_stddev", 1e-2);
    double vel_noise_stddev = declare_parameter("vel_noise_stddev", 1e-2);
    double rpy_noise_stddev = declare_parameter("rpy_noise_stddev", 1e-4);
    double steer_noise_stddev = declare_parameter("steer_noise_stddev", 1e-4);
    m.pos_dist_ = std::make_shared<std::normal_distribution<>>(0.0, pos_noise_stddev);
    m.vel_dist_ = std::make_shared<std::normal_distribution<>>(0.0, vel_noise_stddev);
    m.rpy_dist_ = std::make_shared<std::normal_distribution<>>(0.0, rpy_noise_stddev);
    m.steer_dist_ = std::make_shared<std::normal_distribution<>>(0.0, steer_noise_stddev);

    x_stddev_ = declare_parameter("x_stddev", 0.0001);
    y_stddev_ = declare_parameter("y_stddev", 0.0001);
  }

  // control mode
  current_control_mode_.mode = ControlModeReport::AUTONOMOUS;
}

void SimplePlanningSimulator::initialize_vehicle_model(const std::string & vehicle_model_type_str)
{
  const double vel_lim = declare_parameter("vel_lim", 50.0);
  const double vel_rate_lim = declare_parameter("vel_rate_lim", 7.0);
  const double steer_lim = declare_parameter("steer_lim", 1.0);
  const double steer_rate_lim = declare_parameter("steer_rate_lim", 5.0);
  const double acc_time_delay = declare_parameter("acc_time_delay", 0.1);
  const double acc_time_constant = declare_parameter("acc_time_constant", 0.1);
  const double vel_time_delay = declare_parameter("vel_time_delay", 0.25);
  const double vel_time_constant = declare_parameter("vel_time_constant", 0.5);
  const double steer_time_delay = declare_parameter("steer_time_delay", 0.24);
  const double steer_time_constant = declare_parameter("steer_time_constant", 0.27);
  const double steer_dead_band = declare_parameter("steer_dead_band", 0.0);
  const double steer_bias = declare_parameter("steer_bias", 0.0);

  const double debug_acc_scaling_factor = declare_parameter("debug_acc_scaling_factor", 1.0);
  const double debug_steer_scaling_factor = declare_parameter("debug_steer_scaling_factor", 1.0);
  const auto vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();
  const double wheelbase = vehicle_info.wheel_base_m;

  std::vector<std::string> model_module_paths = declare_parameter<std::vector<std::string>>(
    "model_module_paths", std::vector<std::string>({""}));
  std::vector<std::string> model_param_paths = declare_parameter<std::vector<std::string>>(
    "model_param_paths", std::vector<std::string>({""}));
  std::vector<std::string> model_class_names = declare_parameter<std::vector<std::string>>(
    "model_class_names", std::vector<std::string>({""}));

  if (vehicle_model_type_str == "IDEAL_STEER_VEL") {
    vehicle_model_type_ = VehicleModelType::IDEAL_STEER_VEL;
    vehicle_model_ptr_ = std::make_shared<SimModelIdealSteerVel>(wheelbase);
  } else if (vehicle_model_type_str == "IDEAL_STEER_ACC") {
    vehicle_model_type_ = VehicleModelType::IDEAL_STEER_ACC;
    vehicle_model_ptr_ = std::make_shared<SimModelIdealSteerAcc>(wheelbase);
  } else if (vehicle_model_type_str == "IDEAL_STEER_ACC_GEARED") {
    vehicle_model_type_ = VehicleModelType::IDEAL_STEER_ACC_GEARED;
    vehicle_model_ptr_ = std::make_shared<SimModelIdealSteerAccGeared>(wheelbase);
  } else if (vehicle_model_type_str == "DELAY_STEER_VEL") {
    vehicle_model_type_ = VehicleModelType::DELAY_STEER_VEL;
    vehicle_model_ptr_ = std::make_shared<SimModelDelaySteerVel>(
      vel_lim, steer_lim, vel_rate_lim, steer_rate_lim, wheelbase, timer_sampling_time_ms_ / 1000.0,
      vel_time_delay, vel_time_constant, steer_time_delay, steer_time_constant, steer_dead_band,
      steer_bias);
  } else if (vehicle_model_type_str == "DELAY_STEER_ACC") {
    vehicle_model_type_ = VehicleModelType::DELAY_STEER_ACC;
    vehicle_model_ptr_ = std::make_shared<SimModelDelaySteerAcc>(
      vel_lim, steer_lim, vel_rate_lim, steer_rate_lim, wheelbase, timer_sampling_time_ms_ / 1000.0,
      acc_time_delay, acc_time_constant, steer_time_delay, steer_time_constant, steer_dead_band,
      steer_bias, debug_acc_scaling_factor, debug_steer_scaling_factor);
  } else if (vehicle_model_type_str == "DELAY_STEER_ACC_GEARED") {
    vehicle_model_type_ = VehicleModelType::DELAY_STEER_ACC_GEARED;
    vehicle_model_ptr_ = std::make_shared<SimModelDelaySteerAccGeared>(
      vel_lim, steer_lim, vel_rate_lim, steer_rate_lim, wheelbase, timer_sampling_time_ms_ / 1000.0,
      acc_time_delay, acc_time_constant, steer_time_delay, steer_time_constant, steer_dead_band,
      steer_bias, debug_acc_scaling_factor, debug_steer_scaling_factor);
  } else if (vehicle_model_type_str == "DELAY_STEER_ACC_GEARED_WO_FALL_GUARD") {
    vehicle_model_type_ = VehicleModelType::DELAY_STEER_ACC_GEARED_WO_FALL_GUARD;
    vehicle_model_ptr_ = std::make_shared<SimModelDelaySteerAccGearedWoFallGuard>(
      vel_lim, steer_lim, vel_rate_lim, steer_rate_lim, wheelbase, timer_sampling_time_ms_ / 1000.0,
      acc_time_delay, acc_time_constant, steer_time_delay, steer_time_constant, steer_dead_band,
      steer_bias, debug_acc_scaling_factor, debug_steer_scaling_factor);
  } else if (vehicle_model_type_str == "DELAY_STEER_MAP_ACC_GEARED") {
    vehicle_model_type_ = VehicleModelType::DELAY_STEER_MAP_ACC_GEARED;
    const std::string acceleration_map_path =
      declare_parameter<std::string>("acceleration_map_path");
    if (!std::filesystem::exists(acceleration_map_path)) {
      throw std::runtime_error(
        "`acceleration_map_path` parameter is necessary for `DELAY_STEER_MAP_ACC_GEARED` simulator "
        "model, but " +
        acceleration_map_path +
        " does not exist. Please confirm that the parameter is set correctly in "
        "{simulator_model.param.yaml}.");
    }
    vehicle_model_ptr_ = std::make_shared<SimModelDelaySteerMapAccGeared>(
      vel_lim, steer_lim, vel_rate_lim, steer_rate_lim, wheelbase, timer_sampling_time_ms_ / 1000.0,
      acc_time_delay, acc_time_constant, steer_time_delay, steer_time_constant, steer_bias,
      acceleration_map_path);
  } else if (vehicle_model_type_str == "LEARNED_STEER_VEL") {
    vehicle_model_type_ = VehicleModelType::LEARNED_STEER_VEL;

    vehicle_model_ptr_ = std::make_shared<SimModelLearnedSteerVel>(
      timer_sampling_time_ms_ / 1000.0, model_module_paths, model_param_paths, model_class_names);
  } else if (vehicle_model_type_str == "ACTUATION_CMD") {
    vehicle_model_type_ = VehicleModelType::ACTUATION_CMD;

    // time delay
    const double accel_time_delay = declare_parameter<double>("accel_time_delay");
    const double accel_time_constant = declare_parameter<double>("accel_time_constant");
    const double brake_time_delay = declare_parameter<double>("brake_time_delay");
    const double brake_time_constant = declare_parameter<double>("brake_time_constant");

    // command conversion flag
    const bool convert_accel_cmd = declare_parameter<bool>("convert_accel_cmd");
    const bool convert_brake_cmd = declare_parameter<bool>("convert_brake_cmd");
    const bool convert_steer_cmd = declare_parameter<bool>("convert_steer_cmd");

    // actuation conversion map
    const std::string accel_map_path = declare_parameter<std::string>("accel_map_path");
    const std::string brake_map_path = declare_parameter<std::string>("brake_map_path");

    // init vehicle model depending on convert_steer_cmd_method
    if (convert_steer_cmd) {
      const std::string convert_steer_cmd_method =
        declare_parameter<std::string>("convert_steer_cmd_method");
      if (convert_steer_cmd_method == "vgr") {
        const double vgr_coef_a = declare_parameter<double>("vgr_coef_a");
        const double vgr_coef_b = declare_parameter<double>("vgr_coef_b");
        const double vgr_coef_c = declare_parameter<double>("vgr_coef_c");
        vehicle_model_ptr_ = std::make_shared<SimModelActuationCmd>(
          vel_lim, steer_lim, vel_rate_lim, steer_rate_lim, wheelbase,
          timer_sampling_time_ms_ / 1000.0, accel_time_delay, accel_time_constant, brake_time_delay,
          brake_time_constant, steer_time_delay, steer_time_constant, steer_bias, convert_accel_cmd,
          convert_brake_cmd, convert_steer_cmd, accel_map_path, brake_map_path, vgr_coef_a,
          vgr_coef_b, vgr_coef_c);
      } else if (convert_steer_cmd_method == "steer_map") {
        const std::string steer_map_path = declare_parameter<std::string>("steer_map_path");
        vehicle_model_ptr_ = std::make_shared<SimModelActuationCmd>(
          vel_lim, steer_lim, vel_rate_lim, steer_rate_lim, wheelbase,
          timer_sampling_time_ms_ / 1000.0, accel_time_delay, accel_time_constant, brake_time_delay,
          brake_time_constant, steer_time_delay, steer_time_constant, steer_bias, convert_accel_cmd,
          convert_brake_cmd, convert_steer_cmd, accel_map_path, brake_map_path, steer_map_path);
      } else {
        throw std::invalid_argument(
          "Invalid convert_steer_cmd_method: " + convert_steer_cmd_method);
      }
    }
  } else {
    throw std::invalid_argument("Invalid vehicle_model_type: " + vehicle_model_type_str);
  }
}

void SimplePlanningSimulator::on_timer()
{
  if (!is_initialized_) {
    publish_control_mode_report();
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "waiting initialization...");
    return;
  }

  // calculate longitudinal acceleration by slope
  constexpr double gravity_acceleration = -9.81;
  const double ego_pitch_angle =0.0;//后面需要的话自己设置，或者动态读取
  const double slope_angle = enable_road_slope_simulation_ ? -ego_pitch_angle : 0.0;
  const double acc_by_slope = gravity_acceleration * std::sin(slope_angle);

  // update vehicle dynamics
  {
    const double dt = delta_time_.get_dt(get_clock()->now());
    vehicle_model_ptr_->setGear(current_gear_cmd_.command);
    set_input(current_input_command_, acc_by_slope);
    vehicle_model_ptr_->update(dt);
  }
  // set current state
  const auto prev_odometry = current_odometry_;
  current_odometry_ = to_odometry(vehicle_model_ptr_, ego_pitch_angle);
  current_velocity_ = to_velocity_report(vehicle_model_ptr_);
  current_steer_ = to_steering_report(vehicle_model_ptr_);

  if (add_measurement_noise_) {
    add_measurement_noise(current_odometry_, current_velocity_, current_steer_);
  }
  // add estimate covariance
  {
    using COV_IDX = autoware::universe_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
    current_odometry_.pose.covariance[COV_IDX::X_X] = x_stddev_;
    current_odometry_.pose.covariance[COV_IDX::Y_Y] = y_stddev_;
  }
  // publish vehicle state
  publish_odometry(current_odometry_);
  publish_pose(current_odometry_);
  publish_velocity(current_velocity_);
  publish_acceleration();
  publish_imu();
  publish_steering(current_steer_);

  publish_control_mode_report();
  publish_gear_report();
  publish_turn_indicators_report();//如果没接收到，就不发布
  publish_hazard_lights_report();//如果没接收到，就不发布
  publish_tf(current_odometry_);

  if (vehicle_model_ptr_->shouldPublishActuationStatus()) {
    publish_actuation_status();
  }

}
void SimplePlanningSimulator::set_map_baselink_transforms(const Odometry & odometry)
{
    geometry_msgs::msg::TransformStamped t;
  
    t.header.stamp = get_clock()->now();
    t.header.frame_id = origin_frame_id_;
    t.child_frame_id = simulated_frame_id_;

    t.transform.translation.x = odometry.pose.pose.position.x;
    t.transform.translation.y = odometry.pose.pose.position.y;
    t.transform.translation.z = odometry.pose.pose.position.z;
    t.transform.rotation.x = odometry.pose.pose.orientation.x;
    t.transform.rotation.y = odometry.pose.pose.orientation.y;
    t.transform.rotation.z = odometry.pose.pose.orientation.z;
    t.transform.rotation.w = odometry.pose.pose.orientation.w;
  
    tf_static_broadcaster_->sendTransform(t);
}

void SimplePlanningSimulator::set_input(const InputCommand & cmd, const double acc_by_slope)
{
  std::visit(
    [this, acc_by_slope](auto && arg) {
      using T = std::decay_t<decltype(arg)>;
      if constexpr (std::is_same_v<T, Control>) {
        set_input(arg, acc_by_slope);
      } else if constexpr (std::is_same_v<T, ActuationCommandStamped>) {
        set_input(arg, acc_by_slope);
      } else {
        throw std::invalid_argument("Invalid input command type");
      }
    },
    cmd);
}

void SimplePlanningSimulator::set_input(
  const ActuationCommandStamped & cmd, const double acc_by_slope)
{
  const auto accel = cmd.actuation.accel_cmd;
  const auto brake = cmd.actuation.brake_cmd;
  const auto steer = cmd.actuation.steer_cmd;
  const auto gear = vehicle_model_ptr_->getGear();

  Eigen::VectorXd input(vehicle_model_ptr_->getDimU());
  input << accel, brake, acc_by_slope, steer, gear;

  // VehicleModelType::ACTUATION_COMMAND
  vehicle_model_ptr_->setInput(input);
}

void SimplePlanningSimulator::set_input(const Control & cmd, const double acc_by_slope)
{
  const auto steer = cmd.lateral.steering_tire_angle;
  const auto vel = cmd.longitudinal.velocity;
  const auto acc_by_cmd = cmd.longitudinal.acceleration;

  using autoware_vehicle_msgs::msg::GearCommand;
  Eigen::VectorXd input(vehicle_model_ptr_->getDimU());
  const auto gear = vehicle_model_ptr_->getGear();

  // TODO(Watanabe): The definition of the sign of acceleration in REVERSE mode is different
  // between .auto and proposal.iv, and will be discussed later.
  const float combined_acc = [&] {
    if (gear == GearCommand::NONE) {
      return 0.0;
    } else if (gear == GearCommand::REVERSE || gear == GearCommand::REVERSE_2) {
      return -acc_by_cmd + acc_by_slope;
    } else {
      return acc_by_cmd + acc_by_slope;
    }
  }();

  if (
    vehicle_model_type_ == VehicleModelType::IDEAL_STEER_VEL ||
    vehicle_model_type_ == VehicleModelType::DELAY_STEER_VEL ||
    vehicle_model_type_ == VehicleModelType::LEARNED_STEER_VEL) {
    input << vel, steer;
  } else if (  // NOLINT
    vehicle_model_type_ == VehicleModelType::IDEAL_STEER_ACC ||
    vehicle_model_type_ == VehicleModelType::DELAY_STEER_ACC) {
    input << combined_acc, steer;
  } else if (  // NOLINT
    vehicle_model_type_ == VehicleModelType::IDEAL_STEER_ACC_GEARED ||
    vehicle_model_type_ == VehicleModelType::DELAY_STEER_ACC_GEARED ||
    vehicle_model_type_ == VehicleModelType::DELAY_STEER_MAP_ACC_GEARED) {
    input << combined_acc, steer;
  } else if (  // NOLINT
    vehicle_model_type_ == VehicleModelType::DELAY_STEER_ACC_GEARED_WO_FALL_GUARD) {
    input << acc_by_cmd, gear, acc_by_slope, steer;
  }
  vehicle_model_ptr_->setInput(input);
}

void SimplePlanningSimulator::on_turn_indicators_cmd(
  const TurnIndicatorsCommand::ConstSharedPtr msg)
{
  current_turn_indicators_cmd_ptr_ = msg;
}

void SimplePlanningSimulator::on_hazard_lights_cmd(const HazardLightsCommand::ConstSharedPtr msg)
{
  current_hazard_lights_cmd_ptr_ = msg;
}


void SimplePlanningSimulator::add_measurement_noise(
  Odometry & odom, VelocityReport & vel, SteeringReport & steer) const
{
  auto & n = measurement_noise_;
  odom.pose.pose.position.x += (*n.pos_dist_)(*n.rand_engine_);
  odom.pose.pose.position.y += (*n.pos_dist_)(*n.rand_engine_);
  const auto velocity_noise = (*n.vel_dist_)(*n.rand_engine_);
  odom.twist.twist.linear.x += velocity_noise;
  double yaw = tf2::getYaw(odom.pose.pose.orientation);
  yaw += static_cast<float>((*n.rpy_dist_)(*n.rand_engine_));
  odom.pose.pose.orientation = autoware::universe_utils::createQuaternionFromYaw(yaw);

  vel.longitudinal_velocity += static_cast<double>(velocity_noise);

  steer.steering_tire_angle += static_cast<double>((*n.steer_dist_)(*n.rand_engine_));
}

void SimplePlanningSimulator::set_initial_state(const Pose & pose, const Twist & twist)
{
  const double x = pose.position.x;
  const double y = pose.position.y;
  const double yaw = tf2::getYaw(pose.orientation);
  const double yaw_rate = 0.0;
  const double vx = twist.linear.x;
  const double vy = 0.0;
  const double steer = 0.0;
  const double accx = 0.0;

  Eigen::VectorXd state(vehicle_model_ptr_->getDimX());

  if (vehicle_model_type_ == VehicleModelType::IDEAL_STEER_VEL) {
    state << x, y, yaw;
  } else if (  // NOLINT
    vehicle_model_type_ == VehicleModelType::IDEAL_STEER_ACC ||
    vehicle_model_type_ == VehicleModelType::IDEAL_STEER_ACC_GEARED) {
    state << x, y, yaw, vx;
  } else if (  // NOLINT
    vehicle_model_type_ == VehicleModelType::DELAY_STEER_VEL) {
    state << x, y, yaw, vx, steer;
  } else if (vehicle_model_type_ == VehicleModelType::LEARNED_STEER_VEL) {
    state << x, y, yaw, yaw_rate, vx, vy, steer;
  } else if (  // NOLINT
    vehicle_model_type_ == VehicleModelType::DELAY_STEER_ACC ||
    vehicle_model_type_ == VehicleModelType::DELAY_STEER_ACC_GEARED ||
    vehicle_model_type_ == VehicleModelType::DELAY_STEER_ACC_GEARED_WO_FALL_GUARD ||
    vehicle_model_type_ == VehicleModelType::DELAY_STEER_MAP_ACC_GEARED ||
    vehicle_model_type_ == VehicleModelType::ACTUATION_CMD) {
    state << x, y, yaw, vx, steer, accx;
  }
  vehicle_model_ptr_->setState(state);

  is_initialized_ = true;
}


void SimplePlanningSimulator::publish_odometry(const Odometry & odometry)
{
  Odometry msg = odometry;
  msg.header.frame_id = origin_frame_id_;
  msg.header.stamp = get_clock()->now();
  msg.child_frame_id = simulated_frame_id_;
  pub_odom_->publish(msg);
}

void SimplePlanningSimulator::publish_pose(const Odometry & odometry)
{
  geometry_msgs::msg::PoseWithCovarianceStamped msg;

  msg.pose = odometry.pose;
  using COV_IDX = autoware::universe_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
  constexpr auto COV_POS = 0.0225;      // same value as current ndt output
  constexpr auto COV_ANGLE = 0.000625;  // same value as current ndt output
  msg.pose.covariance.at(COV_IDX::X_X) = COV_POS;
  msg.pose.covariance.at(COV_IDX::Y_Y) = COV_POS;
  msg.pose.covariance.at(COV_IDX::Z_Z) = COV_POS;
  msg.pose.covariance.at(COV_IDX::ROLL_ROLL) = COV_ANGLE;
  msg.pose.covariance.at(COV_IDX::PITCH_PITCH) = COV_ANGLE;
  msg.pose.covariance.at(COV_IDX::YAW_YAW) = COV_ANGLE;

  msg.header.frame_id = origin_frame_id_;
  msg.header.stamp = get_clock()->now();
  pub_current_pose_->publish(msg);
}


void SimplePlanningSimulator::publish_velocity(const VelocityReport & velocity)
{
  VelocityReport msg = velocity;
  msg.header.stamp = get_clock()->now();
  msg.header.frame_id = simulated_frame_id_;
  pub_velocity_->publish(msg);
}

void SimplePlanningSimulator::publish_acceleration()
{
  AccelWithCovarianceStamped msg;
  msg.header.frame_id = "/base_link";
  msg.header.stamp = get_clock()->now();
  msg.accel.accel.linear.x = vehicle_model_ptr_->getAx();
  msg.accel.accel.linear.y = vehicle_model_ptr_->getWz() * vehicle_model_ptr_->getVx();

  using COV_IDX = autoware::universe_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
  constexpr auto COV = 0.001;
  msg.accel.covariance.at(COV_IDX::X_X) = COV;          // linear x
  msg.accel.covariance.at(COV_IDX::Y_Y) = COV;          // linear y
  msg.accel.covariance.at(COV_IDX::Z_Z) = COV;          // linear z
  msg.accel.covariance.at(COV_IDX::ROLL_ROLL) = COV;    // angular x
  msg.accel.covariance.at(COV_IDX::PITCH_PITCH) = COV;  // angular y
  msg.accel.covariance.at(COV_IDX::YAW_YAW) = COV;      // angular z
  pub_acc_->publish(msg);
}

void SimplePlanningSimulator::publish_imu()
{
  using COV_IDX = autoware::universe_utils::xyz_covariance_index::XYZ_COV_IDX;

  sensor_msgs::msg::Imu imu;
  imu.header.frame_id = "base_link";
  imu.header.stamp = now();
  imu.linear_acceleration.x = vehicle_model_ptr_->getAx();
  imu.linear_acceleration.y = vehicle_model_ptr_->getWz() * vehicle_model_ptr_->getVx();
  constexpr auto COV = 0.001;
  imu.linear_acceleration_covariance.at(COV_IDX::X_X) = COV;
  imu.linear_acceleration_covariance.at(COV_IDX::Y_Y) = COV;
  imu.linear_acceleration_covariance.at(COV_IDX::Z_Z) = COV;
  imu.angular_velocity = current_odometry_.twist.twist.angular;
  imu.angular_velocity_covariance.at(COV_IDX::X_X) = COV;
  imu.angular_velocity_covariance.at(COV_IDX::Y_Y) = COV;
  imu.angular_velocity_covariance.at(COV_IDX::Z_Z) = COV;
  imu.orientation = current_odometry_.pose.pose.orientation;
  imu.orientation_covariance.at(COV_IDX::X_X) = COV;
  imu.orientation_covariance.at(COV_IDX::Y_Y) = COV;
  imu.orientation_covariance.at(COV_IDX::Z_Z) = COV;
  pub_imu_->publish(imu);
}

void SimplePlanningSimulator::publish_steering(const SteeringReport & steer)
{
  SteeringReport msg = steer;
  msg.stamp = get_clock()->now();
  pub_steer_->publish(msg);
}

void SimplePlanningSimulator::publish_control_mode_report()
{
  current_control_mode_.stamp = get_clock()->now();
  pub_control_mode_report_->publish(current_control_mode_);
}

void SimplePlanningSimulator::publish_gear_report()
{
  GearReport msg;
  msg.stamp = get_clock()->now();
  msg.report = vehicle_model_ptr_->getGear();
  pub_gear_report_->publish(msg);
}

void SimplePlanningSimulator::publish_turn_indicators_report()
{
  if (!current_turn_indicators_cmd_ptr_) {
    return;
  }
  TurnIndicatorsReport msg;
  msg.stamp = get_clock()->now();
  msg.report = current_turn_indicators_cmd_ptr_->command;
  pub_turn_indicators_report_->publish(msg);
}

void SimplePlanningSimulator::publish_hazard_lights_report()
{
  if (!current_hazard_lights_cmd_ptr_) {
    return;
  }
  HazardLightsReport msg;
  msg.stamp = get_clock()->now();
  msg.report = current_hazard_lights_cmd_ptr_->command;
  pub_hazard_lights_report_->publish(msg);
}

void SimplePlanningSimulator::publish_tf(const Odometry & odometry)
{
  TransformStamped tf;
  tf.header.stamp = get_clock()->now();
  tf.header.frame_id = origin_frame_id_;
  tf.child_frame_id = simulated_frame_id_;
  tf.transform.translation.x = odometry.pose.pose.position.x;
  tf.transform.translation.y = odometry.pose.pose.position.y;
  tf.transform.translation.z = odometry.pose.pose.position.z;
  tf.transform.rotation = odometry.pose.pose.orientation;

  tf2_msgs::msg::TFMessage tf_msg{};
  tf_msg.transforms.emplace_back(std::move(tf));
  pub_tf_->publish(tf_msg);
}

void SimplePlanningSimulator::publish_actuation_status()
{
  auto actuation_status = vehicle_model_ptr_->getActuationStatus();
  if (!actuation_status.has_value()) {
    return;
  }

  actuation_status.value().header.stamp = get_clock()->now();
  actuation_status.value().header.frame_id = simulated_frame_id_;
  pub_actuation_status_->publish(actuation_status.value());
}


}  // namespace simple_planning_simulator
}  // namespace simulation

RCLCPP_COMPONENTS_REGISTER_NODE(simulation::simple_planning_simulator::SimplePlanningSimulator)
