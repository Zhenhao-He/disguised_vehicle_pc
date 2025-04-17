#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/point.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <tf2_ros/buffer.h>
#include "nav_msgs/msg/odometry.hpp"
#include <tf2_ros/transform_broadcaster.h>

using nav_msgs::msg::Odometry;

#define M_PI 3.14159265358979323846
class CoordinateTransformNode : public rclcpp::Node
{
public:
    CoordinateTransformNode() : Node("coordinate_transform_node")
    {
        using rclcpp::QoS;
        using std::placeholders::_1;
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        gnss_utm_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "~/input/UTM", rclcpp::QoS(1).best_effort(), 
            std::bind(&CoordinateTransformNode::on_gnss_utm, this, std::placeholders::_1));

        pub_position = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/output/position", 10);
        // pub_base_link_odometry_ = create_publisher<Odometry>("~/output/base_link_odometry_", QoS{1});
        // pub_rtk_odometry_ = create_publisher<Odometry>("~/output/rtk_odometry_", QoS{1});
        // timer_ = this->create_wall_timer(
        //     std::chrono::milliseconds(100), std::bind(&CoordinateTransformNode::timer_callback, this));

    }
    
private:
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    geometry_msgs::msg::TransformStamped transform_stamped;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gnss_utm_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_position;
    rclcpp::Publisher<Odometry>::SharedPtr pub_base_link_odometry_;
    rclcpp::Publisher<Odometry>::SharedPtr pub_rtk_odometry_;
    int stop_flag=0;

    void on_gnss_utm(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        if (stop_flag<10)
        {
            tf2::Quaternion quat;
            tf2::fromMsg(msg->pose.pose.orientation, quat);
            double roll, pitch, yaw;
            tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            sendTransform("base_link", "rtk", 0, 0, yaw, msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
            transform_stamped=getTransformMatrix("rtk", "base_link");  
            stop_flag++;
        }
        else
        {       
            Odometry rtk_odometry;
            rtk_odometry.header.frame_id = "rtk";
            rtk_odometry.pose.pose = msg->pose.pose;

            Odometry base_link_odometry = Transform_Odometry(transform_stamped, rtk_odometry, "rtk", "base_link");

            double x=base_link_odometry.pose.pose.position.x;
            double y=base_link_odometry.pose.pose.position.y;

            base_link_odometry.pose.pose.position.x=y;
            base_link_odometry.pose.pose.position.y=-x;
            base_link_odometry.pose.pose.position.z=0;
            // pub_base_link_odometry_->publish(base_link_odometry);
            // pub_rtk_odometry_->publish(rtk_odometry);

            geometry_msgs::msg::PoseStamped  position;
            position.header.frame_id = "base_link";
            position.pose.position=base_link_odometry.pose.pose.position;
            position.pose.orientation=base_link_odometry.pose.pose.orientation;
            pub_position->publish(position);

            // std::cout << "base_link_odometry.pose.pose.position.x" << base_link_odometry.pose.pose.position.x <<":\n";
            // std::cout << "base_link_odometry_.pose.pose.position.y" << base_link_odometry.pose.pose.position.y <<":\n"; 
            // std::cout << "base_link_odometry.pose.pose.position.z" << base_link_odometry.pose.pose.position.z <<":\n";       
            
            // std::cout << "position.pose.position.x" << position.pose.position.x <<":\n";
            // std::cout << "position.pose.position.y" << position.pose.position.y <<":\n"; 
            // std::cout << "position.pose.position.z" << position.pose.position.z <<":\n";   
        }
    }
    // 封装的函数，广播坐标变换
    void sendTransform(
        const std::string& parent_frame_id,
        const std::string& child_frame_id,
        double pitch, // 单位：弧度
        double roll,  // 单位：弧度
        double yaw,   // 单位：弧度
        double x, double y, double z// 单位：米
    ) {
        // 创建 TransformStamped 消息
        geometry_msgs::msg::TransformStamped transform_msg;
        transform_msg.header.stamp = get_clock()->now();
        transform_msg.header.frame_id = parent_frame_id;
        transform_msg.child_frame_id = child_frame_id;

        // 设置平移部分
        transform_msg.transform.translation.x = x;  
        transform_msg.transform.translation.y = y;
        transform_msg.transform.translation.z = z;

        // 创建并设置旋转部分
        tf2::Quaternion quaternion;
        quaternion.setRPY(roll, pitch, yaw);  // 输入欧拉角：roll, pitch, yaw (单位：弧度)
        transform_msg.transform.rotation=tf2::toMsg(quaternion);

        // 广播变换
        tf_static_broadcaster_->sendTransform(transform_msg);
    }

    geometry_msgs::msg::TransformStamped getTransformMatrix(
        const std::string& target_frame, 
        const std::string& source_frame)
    {
        // 创建tf2缓冲区和监听器
        tf2_ros::Buffer tf_buffer(get_clock());
        tf2_ros::TransformListener tf_listener(tf_buffer);

        try {
            // 查询变换
            geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer.lookupTransform(
                target_frame, source_frame, rclcpp::Time(0), rclcpp::Duration(1, 0));
            

            //打印获取到的变换
            std::cout << "Transform from " << source_frame << " to " << target_frame << ":\n";
            std::cout << "Translation: [" 
                    << transform_stamped.transform.translation.x << ", " 
                    << transform_stamped.transform.translation.y << ", " 
                    << transform_stamped.transform.translation.z << "]\n";

            // 获取四元数
            tf2::Quaternion quaternion;
            tf2::fromMsg(transform_stamped.transform.rotation, quaternion);

            // 将四元数转换为旋转矩阵
            tf2::Matrix3x3 rotation_matrix(quaternion);

            // 输出旋转矩阵
            double roll, pitch, yaw;
            rotation_matrix.getRPY(roll, pitch, yaw);
            std::cout << "Rotation (RPY): [" << roll << ", " << pitch << ", " << yaw << "]\n";
            
            // 如果需要旋转矩阵，你也可以直接获取它
            // std::cout << "Rotation Matrix:\n";
            // for (int i = 0; i < 3; i++) {
            //     for (int j = 0; j < 3; j++) {
            //         std::cout << rotation_matrix[i][j] << " ";
            //     }
            //     std::cout << std::endl;
            // }
            return transform_stamped;
        }
        catch (const tf2::TransformException &ex) {
            std::cerr << "Error getting transform: " << ex.what() << std::endl;
        }
        
    }

    Odometry Transform_Odometry(
        geometry_msgs::msg::TransformStamped transform_stamped,
        Odometry odometry_rtk,
        const std::string& rtk_frame, 
        const std::string& base_link_frame)
    {
        // 使用 tf2::doTransform 来进行坐标变换
        tf2::Transform transform;
        tf2::fromMsg(transform_stamped.transform, transform);  // 从TransformStamped获取tf2::Transform

        tf2::Quaternion rotation = transform.getRotation();
        
        // 提取平移部分
        tf2::Vector3 translation = transform.getOrigin();

        // ===== 处理位置和姿态（Pose）=====

        Odometry odometry_base_link  = odometry_rtk; // 复制数据
        odometry_base_link.header.frame_id = base_link_frame;

        // 提取原始位姿
        tf2::Vector3 position_base(
            odometry_rtk.pose.pose.position.x,
            odometry_rtk.pose.pose.position.y,
            odometry_rtk.pose.pose.position.z);

        // 变换位置：先旋转，再平移
        tf2::Vector3 position_base_link = tf2::quatRotate(rotation, position_base) + translation;
        odometry_base_link.pose.pose.position.x = position_base_link.x();
        odometry_base_link.pose.pose.position.y = position_base_link.y();
        odometry_base_link.pose.pose.position.z = position_base_link.z();

        // 变换方向（姿态）：四元数旋转
        tf2::Quaternion orientation_base;
        tf2::fromMsg(odometry_rtk.pose.pose.orientation, orientation_base);
        tf2::Quaternion orientation_base_link = rotation * orientation_base;  // 旋转变换
        odometry_base_link.pose.pose.orientation = tf2::toMsg(orientation_base_link);

        return odometry_base_link;
    }
    
    void timer_callback()
    {     
        
    }


    


};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoordinateTransformNode>());
    rclcpp::shutdown();
    return 0;
}

