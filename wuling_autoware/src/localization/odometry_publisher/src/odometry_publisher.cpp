#include <rclcpp/rclcpp.hpp>
//#include <geometry_msgs/msg/pose.hpp>
//#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include "autoware_vehicle_msgs/msg/velocity_report.hpp"
#include <nav_msgs/msg/odometry.hpp>


using geometry_msgs::msg::PoseStamped;
using autoware_vehicle_msgs::msg::VelocityReport;
using nav_msgs::msg::Odometry;

class OdometryPublisher : public rclcpp::Node
{
public:
    OdometryPublisher() : Node("odometry_publisher")
    {   
        // 创建订阅器
        sub_position_ = create_subscription<PoseStamped>(
        "~/input/position", rclcpp::QoS{1},
        std::bind(&OdometryPublisher::on_position, this, std::placeholders::_1));

        sub_velocity_ = create_subscription<VelocityReport>(
        "~/input/velocity_state", rclcpp::QoS{1},
        std::bind(&OdometryPublisher::on_velocity, this, std::placeholders::_1));

        // 创建发布器
        pub_odom_ = this->create_publisher<Odometry>("~/output/kinematic_state", 10);

        // 定时器100毫秒发布一次
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&OdometryPublisher::publish_odometry, this));
    }

private:
    void on_position(const PoseStamped::ConstSharedPtr msg)
    {
        // 设置位置 (假设x, y, z为当前坐标)
        current_odometry_.pose.pose.position=msg->pose.position;
        current_odometry_.pose.pose.orientation=msg->pose.orientation;
    }
    void on_velocity(const VelocityReport::ConstSharedPtr msg)
    {
        current_odometry_.twist.twist.linear.x = msg->longitudinal_velocity;
        current_odometry_.twist.twist.linear.y = 0.0; // 设为0
        current_odometry_.twist.twist.linear.z = 0.0; // 设为0

    }

    void publish_odometry()
    {
        // 设置时间戳
        current_odometry_.header.stamp = this->get_clock()->now();
        current_odometry_.header.frame_id = "map";

        pub_odom_->publish(current_odometry_);

        //RCLCPP_INFO(this->get_logger(), "Publishing Odometry message: [x: %f, y: %f, vx: %f]",
                    //current_odometry_.pose.pose.position.x, current_odometry_.pose.pose.position.y, current_odometry_.twist.twist.linear.x);
    }


    // 订阅器
    rclcpp::Subscription<PoseStamped>::SharedPtr sub_position_;
    rclcpp::Subscription<VelocityReport>::SharedPtr sub_velocity_;
    // 发布器
    rclcpp::Publisher<Odometry>::SharedPtr pub_odom_;
    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;

    // 当前的odometry数据
    Odometry current_odometry_{};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryPublisher>());
    rclcpp::shutdown();
    return 0;
}
