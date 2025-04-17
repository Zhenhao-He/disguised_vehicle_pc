// Copyright 2022 TIER IV, Inc.
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

#ifndef COMPATIBILITY__AUTOWARE_STATE_HPP_
#define COMPATIBILITY__AUTOWARE_STATE_HPP_


#include "rclcpp/rclcpp.hpp"
#include <autoware_system_msgs/msg/autoware_state.hpp>
#include "autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp"
#include "autoware_adapi_v1_msgs/msg/route_state.hpp"
#include "autoware_adapi_v1_msgs/msg/motion_state.hpp"
#include "autoware_adapi_v1_msgs/msg/operation_mode_state.hpp"
#include <vector>
#include <string>
using autoware_system_msgs::msg::AutowareState;
using autoware_adapi_v1_msgs::msg::LocalizationInitializationState;
using autoware_adapi_v1_msgs::msg::RouteState;
using autoware_adapi_v1_msgs::msg::MotionState;
using autoware_adapi_v1_msgs::msg::OperationModeState;
namespace default_ad_api
{
class AutowareStateNode : public rclcpp::Node
{
public:
  explicit AutowareStateNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::TimerBase::SharedPtr timer_;
  
  rclcpp::Publisher<AutowareState>::SharedPtr pub_autoware_state_;
  
  rclcpp::Subscription<LocalizationInitializationState>::SharedPtr sub_localization_state_;
  rclcpp::Subscription<RouteState>::SharedPtr sub_planning_state_;
  rclcpp::Subscription<MotionState>::SharedPtr sub_control_state_;
  rclcpp::Subscription<OperationModeState>::SharedPtr sub_operation_state_;

  LocalizationInitializationState localization_state_;
  RouteState planning_state_;
  MotionState control_state_;
  OperationModeState operation_state_;

  void on_timer();
  void on_localization(const LocalizationInitializationState::ConstSharedPtr msg);
  void on_planning(const RouteState::ConstSharedPtr msg);
  void on_control(const MotionState::ConstSharedPtr msg);
  void on_operation(const OperationModeState::ConstSharedPtr msg);
};
}  // namespace default_ad_api

#endif  // COMPATIBILITY__AUTOWARE_STATE_HPP_
