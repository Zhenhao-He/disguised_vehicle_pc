#include "rclcpp/rclcpp.hpp"
#include "autoware_adapi_v1_msgs/msg/operation_mode_state.hpp"

using autoware_adapi_v1_msgs::msg::OperationModeState;

class OperationModeManager : public rclcpp::Node
{
public:
    OperationModeManager()
        : Node("operation_mode_manager")
    {
        publisher_ = this->create_publisher<OperationModeState>("~/output/operation_mode_state", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&OperationModeManager::publishOperationMode, this));

        // 初始化当前操作模式
        m_current_operation_mode_.is_autoware_control_enabled = true;
        m_current_operation_mode_.mode = OperationModeState::AUTONOMOUS;
    }

private:
    void publishOperationMode()
    {
        auto msg = OperationModeState();
        msg.stamp = this->get_clock()->now();
        msg.is_autoware_control_enabled = m_current_operation_mode_.is_autoware_control_enabled;
        msg.mode = m_current_operation_mode_.mode;

        // 设置内容为 is_autoware_control_enabled && mode == AUTONOMOUS
        if (m_current_operation_mode_.is_autoware_control_enabled &&
            m_current_operation_mode_.mode == OperationModeState::AUTONOMOUS)
        {
            RCLCPP_INFO(this->get_logger(), "Autonomous mode is enabled.");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Autonomous mode is not enabled.");
        }

        publisher_->publish(msg);
    }

    rclcpp::Publisher<OperationModeState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    OperationModeState m_current_operation_mode_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OperationModeManager>());
    rclcpp::shutdown();
    return 0;
}
