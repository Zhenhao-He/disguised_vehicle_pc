#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/static_transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"

#include "geometry_msgs/msg/point.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#define M_PI 3.14159265358979323846
class CoordinateTransformNode : public rclcpp::Node
{
public:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_pose_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_base_pose_;
    tf2::Transform goal_transform;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_A;  // 声明Marker发布器
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_B;  // 声明Marker发布器

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
    double front_camera_roll ;
    double front_camera_pitch ;
    double front_camera_yaw ;
    double front_camera_x ;
    double front_camera_y ;
    double front_camera_z ;
    CoordinateTransformNode() : Node("coordinate_transform_node")
    {

        pub_goal_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/output/goal_pose", 10);
        pub_base_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/output/base_pose", 10);
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        
        pub_image_ = this->create_publisher<sensor_msgs::msg::Image>("/output/image_with_point", 10);
        marker_pub_A = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker_A", 10);  // 删除局部变量声明
        marker_pub_B = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker_B", 10);  // 删除局部变量声明

        {//front_camera
            geometry_msgs::msg::TransformStamped t_front_camera;
            t_front_camera.header.stamp = get_clock()->now();
            t_front_camera.header.frame_id = "base_link";
            t_front_camera.child_frame_id = "front_camera";


            front_camera_pitch = 90* M_PI / 180.0;
            front_camera_roll = 0* M_PI / 180.0;
            front_camera_yaw= 00* M_PI / 180.0;

            //  front_camera_pitch = -20.999781536615494* M_PI / 180.0;
            //  front_camera_roll =  -0.26601432139592862* M_PI / 180.0;
            //  front_camera_yaw = 1.392716491159997* M_PI / 180.0;
             front_camera_x = 1700.5524701956288*0.001;
             front_camera_y = -12.853577420775263*0.001;
             front_camera_z = 557.40604512058906*0.001;


            t_front_camera.transform.translation.x = front_camera_x;
            t_front_camera.transform.translation.y = front_camera_y;
            t_front_camera.transform.translation.z = front_camera_z;
            tf2::Quaternion quaternion_front_camera;
            quaternion_front_camera.setRPY(front_camera_roll, front_camera_pitch, front_camera_yaw);  // 输入欧拉角：roll, pitch, yaw (单位：弧度)
            t_front_camera.transform.rotation.x = quaternion_front_camera.x();
            t_front_camera.transform.rotation.y = quaternion_front_camera.y();
            t_front_camera.transform.rotation.z = quaternion_front_camera.z();
            t_front_camera.transform.rotation.w = quaternion_front_camera.w();

            tf_static_broadcaster_->sendTransform(t_front_camera);

        }


        {//back_camera
            geometry_msgs::msg::TransformStamped t_back_camera;
            t_back_camera.header.stamp = get_clock()->now();
            t_back_camera.header.frame_id = "base_link";
            t_back_camera.child_frame_id = "back_camera";

            double back_camera_pitch = -27.103623569549477* M_PI / 180.0;
            double back_camera_roll = 0.90688528351123943* M_PI / 180.0;
            double back_camera_yaw = 1.135238455548728* M_PI / 180.0;
            double back_camera_x = -1670.5408167438618*0.001;
            double back_camera_y = 1.1718054006750904*0.001;
            double back_camera_z = 576.92180476440819*0.001;

            t_back_camera.transform.translation.x = back_camera_x;
            t_back_camera.transform.translation.y = back_camera_y;
            t_back_camera.transform.translation.z = back_camera_z;
            tf2::Quaternion quaternion_back_camera;
            quaternion_back_camera.setRPY(back_camera_roll, back_camera_pitch, back_camera_yaw);  // 输入欧拉角：roll, pitch, yaw (单位：弧度)
            t_back_camera.transform.rotation.x = quaternion_back_camera.x();
            t_back_camera.transform.rotation.y = quaternion_back_camera.y();
            t_back_camera.transform.rotation.z = quaternion_back_camera.z();
            t_back_camera.transform.rotation.w = quaternion_back_camera.w();

            tf_static_broadcaster_->sendTransform(t_back_camera);

        }

        {//left_camera
            geometry_msgs::msg::TransformStamped t_left_camera;
            t_left_camera.header.stamp = get_clock()->now();
            t_left_camera.header.frame_id = "base_link";
            t_left_camera.child_frame_id = "left_camera";

            double left_camera_pitch = -39.19279355763684* M_PI / 180.0;
            double left_camera_roll = -5.9685654282657623* M_PI / 180.0;
            double left_camera_yaw = -7.241107363171106* M_PI / 180.0;
            double left_camera_x = 455.60566814662781*0.001;
            double left_camera_y = -1003.6904701380944*0.001;
            double left_camera_z = 1085.2214333375553*0.001;

            t_left_camera.transform.translation.x = left_camera_x;
            t_left_camera.transform.translation.y = left_camera_y;
            t_left_camera.transform.translation.z = left_camera_z;
            tf2::Quaternion quaternion_left_camera;
            quaternion_left_camera.setRPY(left_camera_roll, left_camera_pitch, left_camera_yaw);  // 输入欧拉角：roll, pitch, yaw (单位：弧度)
            t_left_camera.transform.rotation.x = quaternion_left_camera.x();
            t_left_camera.transform.rotation.y = quaternion_left_camera.y();
            t_left_camera.transform.rotation.z = quaternion_left_camera.z();
            t_left_camera.transform.rotation.w = quaternion_left_camera.w();

            tf_static_broadcaster_->sendTransform(t_left_camera);

        }
        
        {//right_camera
            geometry_msgs::msg::TransformStamped t_right_camera;
            t_right_camera.header.stamp = get_clock()->now();
            t_right_camera.header.frame_id = "base_link";
            t_right_camera.child_frame_id = "right_camera";

            double right_camera_pitch = -35.894672157298132* M_PI / 180.0;
            double right_camera_roll = 1.9779390058529731* M_PI / 180.0;
            double right_camera_yaw = 2.4543808688412212* M_PI / 180.0;
            double right_camera_x = 366.15874478977992*0.001;
            double right_camera_y = 942.65784760151428*0.001;
            double right_camera_z = 1084.7134281747294*0.001;

            t_right_camera.transform.translation.x = right_camera_x;
            t_right_camera.transform.translation.y = right_camera_y;
            t_right_camera.transform.translation.z = right_camera_z;
            tf2::Quaternion quaternion_right_camera;
            quaternion_right_camera.setRPY(right_camera_roll, right_camera_pitch, right_camera_yaw);  // 输入欧拉角：roll, pitch, yaw (单位：弧度)
            t_right_camera.transform.rotation.x = quaternion_right_camera.x();
            t_right_camera.transform.rotation.y = quaternion_right_camera.y();
            t_right_camera.transform.rotation.z = quaternion_right_camera.z();
            t_right_camera.transform.rotation.w = quaternion_right_camera.w();

            tf_static_broadcaster_->sendTransform(t_right_camera);

        }
              
        // 定期更新可视化
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&CoordinateTransformNode::timer_callback, this));
    }
    
private:
    void timer_callback()
    {      
        //3d点转换
        double goal_pitch = front_camera_pitch;
        double goal_roll = front_camera_roll;
        double goal_yaw = front_camera_yaw;
        double goal_x = front_camera_x;
        double goal_y = front_camera_y;
        double goal_z = front_camera_z;
        // 构建旋转四元数
        tf2::Quaternion goal_quat;
        goal_quat.setRPY(goal_roll, goal_pitch, goal_yaw);  // 设置以roll, pitch, yaw的顺序

        // 创建坐标系B相对于坐标系A的变换
        goal_transform.setOrigin(tf2::Vector3(goal_x, goal_y, goal_z));  // 设置平移
        goal_transform.setRotation(goal_quat);  // 设置旋转


        tf2::Transform inverse_transform = goal_transform.inverse();


        // 输入坐标系A下的点（例如：A下的点坐标为(1, 1, 1)）
        geometry_msgs::msg::Point point_in_A;
        point_in_A.x = 2.80;
        point_in_A.y = 1.00;
        point_in_A.z = 0.0;

        // 将点坐标从坐标系A转化到坐标系B
        tf2::Vector3 point_A(point_in_A.x, point_in_A.y, point_in_A.z);
        tf2::Vector3 point_B = inverse_transform * point_A;  // 进行坐标变换

        // 输出坐标系B下的点坐标
        RCLCPP_INFO(this->get_logger(), "Point in A: (%.2f, %.2f, %.2f)", point_in_A.x, point_in_A.y, point_in_A.z);
        RCLCPP_INFO(this->get_logger(), "Point in B: (%.2f, %.2f, %.2f)", point_B.x(), point_B.y(), point_B.z());
        
        
        //可视化point_A
        // 创建一个Marker消息
        visualization_msgs::msg::Marker marker_point_A;
        marker_point_A.header.frame_id = "base_link";  // 在B坐标系下
        marker_point_A.header.stamp = this->get_clock()->now();
        marker_point_A.ns = "points";
        marker_point_A.id = 0;
        marker_point_A.type = visualization_msgs::msg::Marker::SPHERE;
        marker_point_A.action = visualization_msgs::msg::Marker::ADD;
        marker_point_A.pose.position.x = point_A.x();
        marker_point_A.pose.position.y = point_A.y();
        marker_point_A.pose.position.z = point_A.z();
        marker_point_A.pose.orientation.w = 1.0;  // 默认旋转

        marker_point_A.scale.x = 0.1;
        marker_point_A.scale.y = 0.1;
        marker_point_A.scale.z = 0.1;
        marker_point_A.color.r = 1.0f;  // 红色
        marker_point_A.color.g = 0.0f;
        marker_point_A.color.b = 0.0f;
        marker_point_A.color.a = 1.0f;  // 不透明

        // 发布Marker
        marker_pub_A->publish(marker_point_A);

        //可视化point_B
        // 创建一个Marker消息
        visualization_msgs::msg::Marker marker_point_B;
        marker_point_B.header.frame_id = "front_camera";  // 在B坐标系下
        marker_point_B.header.stamp = this->get_clock()->now();
        marker_point_B.ns = "points";
        marker_point_B.id = 0;
        marker_point_B.type = visualization_msgs::msg::Marker::SPHERE;
        marker_point_B.action = visualization_msgs::msg::Marker::ADD;
        marker_point_B.pose.position.x = point_B.x();
        marker_point_B.pose.position.y = point_B.y();
        marker_point_B.pose.position.z = point_B.z();
        marker_point_B.pose.orientation.w = 1.0;  // 默认旋转

        marker_point_B.scale.x = 0.1;
        marker_point_B.scale.y = 0.1;
        marker_point_B.scale.z = 0.1;
        marker_point_B.color.r = 0.0f;  // 红色
        marker_point_B.color.g = 1.0f;
        marker_point_B.color.b = 0.0f;
        marker_point_B.color.a = 1.0f;  // 不透明

        // 发布Marker
        marker_pub_B->publish(marker_point_B);

        // 初始化相机内参和畸变参数
        CameraParams params = {
            316.89753911882968,  // fx
            316.89753911882968,  // fy
            627.95010284911734,  // cx
            478.79436192433741,  // cy
            0.10894763660224048, // k1
            -0.022162485245748754, // k2
            0.0074447044434985303, // p1
            -0.0016800367866348803  // p2
        };
        // 输出像素坐标
        double u, v;

        // 进行坐标映射
        projectToPixel(params, point_B.x(), point_B.y(), point_B.z(), u, v);

        // 打印结果
        std::cout << "Mapped pixel coordinates: (" << u << ", " << v << ")" << std::endl;

        publish_image(u, v);

        
    }

    struct CameraParams {
        double fx, fy, cx, cy;   // 相机内参
        double k1, k2, p1, p2;   // 畸变参数
    };
    
    // 将相机坐标系下的三维点映射到像素坐标系下的函数
    void projectToPixel(const CameraParams& params, double x, double y, double z, double& u, double& v) {
        
        u = (params.fx * x) / z+ params.cx;
        v = (params.fy * y) / z + params.cy; 
        /*

        // 计算归一化像素坐标
        double u_norm = (params.fx * x) / z + params.cx;
        double v_norm = (params.fy * y) / z + params.cy;
        
        // 畸变校正
        double r2 = u_norm * u_norm + v_norm * v_norm;
        double r4 = r2 * r2;
    
        double dx = u_norm * (1 + params.k1 * r2 + params.k2 * r4) + 2 * params.p1 * u_norm * v_norm + params.p2 * (r2 + 2 * u_norm * u_norm);
        double dy = v_norm * (1 + params.k1 * r2 + params.k2 * r4) + 2 * params.p1 * (r2 + 2 * v_norm * v_norm) + params.p2 * u_norm * v_norm;
    
        // 最终的像素坐标
        u = dx;
        v = dy;             */   
    }
    void publish_image(float x ,float y )
    {
        // 加载图像（替换为你自己的图片路径）
        cv::Mat image = cv::imread("/home/parking/hezhenhao/location_test/src/Copy of front_1736825071881.png");
        
        if (image.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load image!");
            return;
        }

        // 在图像上绘制一个点
        // 设定点的位置为(100, 100)位置，颜色为红色，圆点大小为10
        cv::circle(image, cv::Point(x, y), 10, cv::Scalar(0, 0, 255), -1);

        // 发布图像消息
        auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
        pub_image_->publish(*img_msg);
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoordinateTransformNode>());
    rclcpp::shutdown();
    return 0;
}

