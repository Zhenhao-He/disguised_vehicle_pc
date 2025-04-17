#include "autoware_state.hpp"

namespace default_ad_api
{
AutowareStateNode::AutowareStateNode(const rclcpp::NodeOptions & options)
: Node("autoware_state", options)
{
  using std::placeholders::_1;
  
  // 订阅本地化、规划和控制状态
  sub_localization_state_ = create_subscription<LocalizationInitializationState>("~/input/localization_state", rclcpp::QoS(1),
      std::bind(&AutowareStateNode::on_localization, this, _1));
  sub_planning_state_ = create_subscription<RouteState>("~/input/planning_state", rclcpp::QoS(1),
      std::bind(&AutowareStateNode::on_planning, this, _1));
  sub_control_state_ = create_subscription<MotionState>("~/input/control_state", 1,
      std::bind(&AutowareStateNode::on_control, this, _1));
  sub_operation_state_ = create_subscription<OperationModeState>("~/input/operation_state", 1,
      std::bind(&AutowareStateNode::on_operation, this, _1));

  // 发布 Autoware 状态
  pub_autoware_state_ = create_publisher<AutowareState>("~/output/autoware_state", rclcpp::QoS(1));

  const rclcpp::Rate rate(10);  // 10 Hz
  timer_ = rclcpp::create_timer(this, get_clock(), rate.period(), [this]() { on_timer(); });

  // 初始状态
  localization_state_.state = LocalizationInitializationState::UNKNOWN;
  planning_state_.state = RouteState::UNKNOWN;
  control_state_.state = MotionState::UNKNOWN;
}

void AutowareStateNode::on_localization(const LocalizationInitializationState::ConstSharedPtr msg)
{
  localization_state_ = *msg;
}

void AutowareStateNode::on_planning(const RouteState::ConstSharedPtr msg)
{
  planning_state_ = *msg;
}

void AutowareStateNode::on_control(const MotionState::ConstSharedPtr msg)
{
  control_state_ = *msg;
}

void AutowareStateNode::on_operation(const OperationModeState::ConstSharedPtr msg)
{
  operation_state_ = *msg;
}
void AutowareStateNode::on_timer()
{
  bool is_ready = true;

  const auto convert_state = [this, &is_ready]() {
    if (planning_state_.state == RouteState::ARRIVED) {
      RCLCPP_INFO(get_logger(), "AutowareState::ARRIVED_GOAL.");
      return AutowareState::ARRIVED_GOAL;  // 结束，对应P档
    }
/*
    if (localization_state_.state != LocalizationInitializationState::INITIALIZED) {
      RCLCPP_INFO(get_logger(), "Localization module not initialized.");  // 打印消息：定位模块未完成初始化
      is_ready = false;
    }
*/
    if (planning_state_.state != RouteState::SET) {
      RCLCPP_INFO(get_logger(), "Planning module not initialized.");  // 打印消息：规划模块未完成初始化
      is_ready = false;
    }

    if (control_state_.state != MotionState::MOVING) {
      RCLCPP_INFO(get_logger(), "Control module not initialized.");  
      is_ready = false;
    }
    if (!(operation_state_.is_autoware_control_enabled&&
    operation_state_.mode == OperationModeState::AUTONOMOUS)) {
      RCLCPP_INFO(get_logger(), "Operation Mode is not AUTONOMOUS.");  
      is_ready = false;
    }
    if (is_ready) {
      RCLCPP_INFO(get_logger(), "AutowareState::DRIVING.");
      return AutowareState::DRIVING;  // 如果所有模块都准备好了，返回驾驶状态
    } else {
      RCLCPP_INFO(get_logger(), "AutowareState::INITIALIZING.");
      return AutowareState::INITIALIZING;  // 否则返回初始化状态
    }
  };

  AutowareState msg;
  msg.stamp = now();
  msg.state = convert_state();
  pub_autoware_state_->publish(msg);
}

}  // namespace default_ad_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(default_ad_api::AutowareStateNode)
