#include <rclcpp/rclcpp.hpp>


#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/static_transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <tf2_ros/buffer.h>
#include "nav_msgs/msg/odometry.hpp"
using nav_msgs::msg::Odometry;

#define M_PI 3.14159265358979323846
class CoordinateTransformNode : public rclcpp::Node
{
public:

    tf2::Transform goal_transform;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::TransformStamped transform_stamped;

    Odometry base_link_odometry_{};
    Odometry rtk_odometry_{};
    rclcpp::Publisher<Odometry>::SharedPtr pub_base_link_odometry_;
    rclcpp::Publisher<Odometry>::SharedPtr pub_rtk_odometry_;

    CoordinateTransformNode() : Node("coordinate_transform_node")
    {

        using rclcpp::QoS;
        using std::placeholders::_1;
        using std::placeholders::_2;

        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        

        pub_base_link_odometry_ = create_publisher<Odometry>("output/base_link_odometry_", QoS{1});
        pub_rtk_odometry_ = create_publisher<Odometry>("output/rtk_odometry_", QoS{1});
        //增加小数点位数，精度
        sendTransform("base_link","rtk",0,
                                        0,
                                        0,
                                        245519.271*0.0001,
                                        3379080.479*0.0001,
                                        0);//绕y轴

        transform_stamped=getTransformMatrix("rtk", "base_link");    
        

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&CoordinateTransformNode::timer_callback, this));
    }
    
private:
    // 封装的函数，广播坐标变换
    void sendTransform(
        const std::string& parent_frame_id,
        const std::string& child_frame_id,
        double pitch_deg, // 单位：度
        double roll_deg,  // 单位：度
        double yaw_deg,   // 单位：度
        double x, double y, double z
    ) {
        // 将角度转换为弧度
        double pitch = pitch_deg * M_PI / 180.0;
        double roll = roll_deg * M_PI / 180.0;
        double yaw = yaw_deg * M_PI / 180.0;

        // 创建 TransformStamped 消息
        geometry_msgs::msg::TransformStamped transform_msg;
        transform_msg.header.stamp = get_clock()->now();
        transform_msg.header.frame_id = parent_frame_id;
        transform_msg.child_frame_id = child_frame_id;

        // 设置平移部分
        transform_msg.transform.translation.x = x * 0.001;  // 单位：毫米转米
        transform_msg.transform.translation.y = y * 0.001;
        transform_msg.transform.translation.z = z * 0.001;

        // 创建并设置旋转部分
        tf2::Quaternion quaternion;
        quaternion.setRPY(roll, pitch, yaw);  // 输入欧拉角：roll, pitch, yaw (单位：弧度)

        transform_msg.transform.rotation.x = quaternion.x();
        transform_msg.transform.rotation.y = quaternion.y();
        transform_msg.transform.rotation.z = quaternion.z();
        transform_msg.transform.rotation.w = quaternion.w();

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
               


            // 打印获取到的变换
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
            std::cout << "Rotation Matrix:\n";
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    std::cout << rotation_matrix[i][j] << " ";
                }
                std::cout << std::endl;
            }
            return transform_stamped;
        }
        catch (const tf2::TransformException &ex) {
            std::cerr << "Error getting transform: " << ex.what() << std::endl;
        }
        
    }


    Odometry Transform_Odometry(
        geometry_msgs::msg::TransformStamped transform_stamped,
        Odometry odometry_base_link,
        const std::string& rtk_frame, 
        const std::string& base_link_frame)
    {
        // 使用 tf2::doTransform 来进行坐标变换
        tf2::Transform transform;
        tf2::fromMsg(transform_stamped.transform, transform);  // 从TransformStamped获取tf2::Transform

                // 提取旋转部分（四元数）
        tf2::Quaternion rotation = transform.getRotation();
        
        // 提取平移部分
        tf2::Vector3 translation = transform.getOrigin();

        // ===== 处理位置和姿态（Pose）=====
        Odometry odometry_rtk = odometry_base_link; // 复制数据
        odometry_rtk.header.frame_id = rtk_frame;

        // 提取原始位姿
        tf2::Vector3 position_base(
            odometry_base_link.pose.pose.position.x,
            odometry_base_link.pose.pose.position.y,
            odometry_base_link.pose.pose.position.z);

        // 变换位置：先旋转，再平移
        tf2::Vector3 position_rtk = tf2::quatRotate(rotation, position_base) + translation;
        odometry_rtk.pose.pose.position.x = position_rtk.x();
        odometry_rtk.pose.pose.position.y = position_rtk.y();
        odometry_rtk.pose.pose.position.z = position_rtk.z();

        // 变换方向（姿态）：四元数旋转
        tf2::Quaternion orientation_base;
        tf2::fromMsg(odometry_base_link.pose.pose.orientation, orientation_base);
        tf2::Quaternion orientation_rtk = rotation * orientation_base;  // 旋转变换
        odometry_rtk.pose.pose.orientation = tf2::toMsg(orientation_rtk);
        return odometry_rtk;   
    }
    Odometry inverse_Transform_Odometry(
        geometry_msgs::msg::TransformStamped transform_stamped,
        Odometry odometry_rtk,
        const std::string& rtk_frame, 
        const std::string& base_link_frame)
    {
        // 使用 tf2::doTransform 来进行坐标变换
        tf2::Transform transform;
        tf2::fromMsg(transform_stamped.transform, transform);  // 从TransformStamped获取tf2::Transform
        tf2::Transform inverse_transform = transform.inverse();
                // 提取旋转部分（四元数）
        tf2::Quaternion rotation = inverse_transform.getRotation();
        
        // 提取平移部分
        tf2::Vector3 translation = inverse_transform.getOrigin();

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

        // 赋值位置信息
        // base_link_odometry_.header.frame_id = "base_link";
        // base_link_odometry_.pose.pose.position.x = 2.8;
        // base_link_odometry_.pose.pose.position.y = 1.0;
        // base_link_odometry_.pose.pose.position.z = 0.0;
    
        // // 计算 Yaw 角 90° 的四元数
        // double yaw_du=90;
        // double yaw =yaw_du* M_PI /180 ;  // 90 度 = π/2 弧度
        // tf2::Quaternion q;
        // q.setRPY(0, 0, yaw);  // Roll=0, Pitch=0, Yaw=90°
        // base_link_odometry_.pose.pose.orientation.x = q.x();
        // base_link_odometry_.pose.pose.orientation.y = q.y();
        // base_link_odometry_.pose.pose.orientation.z = q.z();
        // base_link_odometry_.pose.pose.orientation.w = q.w();

        // rtk_odometry_=Transform_Odometry(transform_stamped,base_link_odometry_,  "rtk","base_link");

        // pub_base_link_odometry_->publish(base_link_odometry_);
        // pub_rtk_odometry_->publish(rtk_odometry_);


        rtk_odometry_.header.frame_id = "rtk";
        rtk_odometry_.pose.pose.position.x = 2.8;
        rtk_odometry_.pose.pose.position.y = 1.0;
        rtk_odometry_.pose.pose.position.z = 0.0;
    
        // 计算 Yaw 角 90° 的四元数
        double yaw_du=90;
        double yaw =yaw_du* M_PI /180 ;  // 90 度 = π/2 弧度
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);  // Roll=0, Pitch=0, Yaw=90°
        rtk_odometry_.pose.pose.orientation.x = q.x();
        rtk_odometry_.pose.pose.orientation.y = q.y();
        rtk_odometry_.pose.pose.orientation.z = q.z();
        rtk_odometry_.pose.pose.orientation.w = q.w();

        base_link_odometry_=inverse_Transform_Odometry(transform_stamped,rtk_odometry_,  "rtk","base_link");

        pub_base_link_odometry_->publish(base_link_odometry_);
        pub_rtk_odometry_->publish(rtk_odometry_);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoordinateTransformNode>());
    rclcpp::shutdown();
    return 0;
}

