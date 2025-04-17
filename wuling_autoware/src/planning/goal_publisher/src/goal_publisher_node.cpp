#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <parking_slot_detection_msgs/msg/parking_slot_info.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/point.hpp"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <tf2_ros/buffer.h>
#include "nav_msgs/msg/odometry.hpp"
using nav_msgs::msg::Odometry;
class GoalPublisherNode : public rclcpp::Node
{
public:
    GoalPublisherNode() : Node("goal_publisher_node")
    {
        // 创建发布者
        pub_goal_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/output/goal_pose", 10);
        
        // gnss_utm_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        //                         "~/input/UTM", rclcpp::QoS(1).best_effort(), 
        //                         std::bind(&GoalPublisherNode::on_gnss_utm, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(std::chrono::seconds(1),std::bind(&GoalPublisherNode::publish_goal_pose, this));



    }

private:
    void publish_goal_pose()
    {   
        if(stop_flag==0){
            geometry_msgs::msg::TransformStamped transform_stamped=getTransformMatrix("rtk", "base_link");
            Odometry rtk_odometry;
            rtk_odometry.header.frame_id = "rtk";
            rtk_odometry.pose.pose.position.x=245519.44418690662;
            rtk_odometry.pose.pose.position.y=3379080.4958841195;
            rtk_odometry.pose.pose.position.z=13.771483421325684;

            rtk_odometry.pose.pose.orientation.x=0.0022080801785168127;
            rtk_odometry.pose.pose.orientation.y=0.0018212921856195711;
            rtk_odometry.pose.pose.orientation.z=-0.7208367200943148;
            rtk_odometry.pose.pose.orientation.w=0.6930990046453466;


            Odometry base_link_odometry = Transform_Odometry(transform_stamped, rtk_odometry, "rtk", "base_link");

            double x=base_link_odometry.pose.pose.position.x;
            double y=base_link_odometry.pose.pose.position.y;

            base_link_odometry.pose.pose.position.x=y;
            base_link_odometry.pose.pose.position.y=-x;
            base_link_odometry.pose.pose.position.z=0;
            // pub_base_link_odometry_->publish(base_link_odometry);
            // pub_rtk_odometry_->publish(rtk_odometry);

            geometry_msgs::msg::PoseStamped  position;
            position.header.stamp = this->now();
            position.header.frame_id = "map";
            position.pose.position=base_link_odometry.pose.pose.position;
            position.pose.orientation=base_link_odometry.pose.pose.orientation;
            pub_goal_pose_->publish(position);
            stop_flag=1;

        // 发布消息
        // RCLCPP_INFO(this->get_logger(), "Publishing goal pose: [%.2f, %.2f, %.2f]",
        // message.pose.position.x, message.pose.position.y, message.pose.position.z);
        
        }


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
                target_frame, source_frame, rclcpp::Time(0), rclcpp::Duration(10, 0));
            

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
    
    /*    
                    -90度
                    |
                    |
                    |
    +-180度 ----------------> x轴  0度
                    |
                    |
                    |
                    v
                   y轴
                   90度
    -------------------------------------

                    0度
                    x轴
                    ^
                    |
                    |
                    |
    90度 y轴<---------------- -90度
                    |
                    |
                    |
                +-180度
    */
    void onParkingSlotInfo(const parking_slot_detection_msgs::msg::ParkingSlotInfo::SharedPtr msg)
    {

        // 获取第一个停车位中心和朝向
        auto center = msg->center[0];
        float heading_angle = msg->heading_angle[0];

        float position_y=(image_width/2.0-center.x)*factor;
        float position_x=(image_hight/2.0+80-center.y)*factor;

        if(heading_angle>=-180 && heading_angle<=-90)
        {
            heading_angle=-90-heading_angle;
        }
        else
        {
            heading_angle=180-(heading_angle-90);
        }

        if(heading_angle>=0 && heading_angle<=180)
        {
            heading_angle=heading_angle-180;
        }
        else
        {
            heading_angle=heading_angle+180;
        }
        float heading_angle_rad = heading_angle * M_PI / 180.0;


        // 调试日志：输出接收到的停车位中心信息
        //RCLCPP_INFO(this->get_logger(), "Received parking slot center: x = %f, y = %f", center.x, center.y);
        //RCLCPP_INFO(this->get_logger(), "Received heading angle: %f", heading_angle);

        //将heading_angle转换为四元数
        tf2::Quaternion heading_quaternion;
        heading_quaternion.setRPY(0, 0, heading_angle_rad);

        // 创建并填充PoseStamped消息
        auto message = geometry_msgs::msg::PoseStamped();
        message.header.stamp = this->now();
        message.header.frame_id = "map";

        // 设置目标位置（示例）
        message.pose.position.x = position_x;
        message.pose.position.y = position_y;
        message.pose.position.z = 0.0;
        message.pose.orientation.x = heading_quaternion.x();
        message.pose.orientation.y = heading_quaternion.y();
        message.pose.orientation.z = heading_quaternion.z(); 
        message.pose.orientation.w = heading_quaternion.w();


        // 发消息
        RCLCPP_INFO(this->get_logger(), "Publishing goal pose: [%.2f, %.2f, %.2f]",
                    message.pose.position.x, message.pose.position.y, message.pose.position.z);
        pub_goal_pose_->publish(message);
    }
    /*

    void on_gnss_utm(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {   
        if(stop_flag<100)
        {
            stop_flag++;

            // 提取四元数
            tf2::Quaternion quat;
            tf2::fromMsg(msg->pose.pose.orientation, quat);
        
            // 提取 RPY 角度
            double extracted_roll, extracted_pitch, extracted_yaw;
            tf2::Matrix3x3(quat).getRPY(extracted_roll, extracted_pitch, extracted_yaw);
        
            // 记录偏航角
            double x_cor = 245519.271 - msg->pose.pose.position.x;
            double y_cor = msg->pose.pose.position.y-3379080.479;
            double target_yaw=-92.02* M_PI / 180.0;
            double related_yaw =target_yaw -extracted_yaw;

  
            double new_x = x_cor * cos(target_yaw) + y_cor * sin(target_yaw);
            double new_y = -x_cor * sin(target_yaw) + y_cor * cos(target_yaw);



            target_msg.pose.pose.position.x = new_y;
            target_msg.pose.pose.position.y  = -new_x;
            target_msg.pose.pose.position.z  = 0;

            //将heading_angle转换为四元数
            tf2::Quaternion heading_quaternion;
            heading_quaternion.setRPY(0, 0, related_yaw);

            // 创建并填充PoseStamped消息
            auto message = geometry_msgs::msg::PoseStamped();
            message.header.stamp = this->now();
            message.header.frame_id = "map";

            // 设置目标位置（示例）
            message.pose.position.x = target_msg.pose.pose.position.x;
            message.pose.position.y = target_msg.pose.pose.position.y;
            message.pose.position.z = 0.0;
            message.pose.orientation.x = heading_quaternion.x();
            message.pose.orientation.y = heading_quaternion.y();
            message.pose.orientation.z = heading_quaternion.z(); 
            message.pose.orientation.w = heading_quaternion.w();


            // 发消息
            RCLCPP_INFO(this->get_logger(), "Publishing goal pose: [%.2f, %.2f, %.2f]",
                        message.pose.position.x, message.pose.position.y, related_yaw);
            
            pub_goal_pose_->publish(message);

        }
        
        // else
        // {
        //     RCLCPP_INFO(this->get_logger(), "stop publshing !!!!!!!!!!!!!!!!!!!!");
        //     return;
        // }

    }
        */

    
    geometry_msgs::msg::PoseWithCovarianceStamped target_msg;
    int stop_flag=0;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_pose_;
    rclcpp::Subscription<parking_slot_detection_msgs::msg::ParkingSlotInfo>::SharedPtr parking_slot_sub_;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gnss_utm_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    float image_width=1302;
    float image_hight=1472;


    //float image_width=512;
    //float image_hight=512;
    float factor=0.0092;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
