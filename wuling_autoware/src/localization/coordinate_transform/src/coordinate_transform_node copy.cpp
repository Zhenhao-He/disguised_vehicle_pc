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
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <tf2_ros/buffer.h>


#define M_PI 3.14159265358979323846
class CoordinateTransformNode : public rclcpp::Node
{
public:
    struct CameraParams {
        double fx, fy, cx, cy;   // 相机内参
        double k1, k2, p1, p2;   // 畸变参数
    };
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_pose_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_base_pose_;
    tf2::Transform goal_transform;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_A;  // 声明Marker发布器
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_B;  // 声明Marker发布器

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
    geometry_msgs::msg::TransformStamped transform_stamped;
    CoordinateTransformNode() : Node("coordinate_transform_node")
    {

        pub_goal_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/output/goal_pose", 10);
        pub_base_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/output/base_pose", 10);
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        
        pub_image_ = this->create_publisher<sensor_msgs::msg::Image>("/output/image_with_point", 10);
        marker_pub_A = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker_A", 10);  // 删除局部变量声明
        marker_pub_B = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker_B", 10);  // 删除局部变量声明

        sendTransform("base_link","front_camera_first",90,
                                                        0,
                                                        0,
                                                        1700.5524701956288,
                                                        -12.853577420775263,
                                                        557.40604512058906);//绕y轴

            
                                          
        sendTransform("front_camera_first","front_camera_second",0,
                                                                0,
                                                                -90,
                                                                0,
                                                                0,
                                                                0);//得到标准相机坐标系
        sendTransform("front_camera_second","front_camera_third",0,//偏航
                                                                -20.999781536615494,//俯仰
                                                                0,//横滚
                                                                0,
                                                                0,
                                                                0);//绕x轴
        sendTransform("front_camera_third","front_camera_four",0,//偏航
                                                                0,//俯仰
                                                                0,//横滚
                                                                0,
                                                                0,
                                                                0);//绕x轴
                                                                
        sendTransform("front_camera_four","front_camera_five",0,//偏航
                                                                0,//俯仰
                                                                0,//横滚
                                                                0,
                                                                0,
                                                                0);//绕x轴
        transform_stamped=getTransformMatrix("front_camera_five", "base_link");
            //  front_camera_pitch = -20.999781536615494* M_PI / 180.0;
            //  front_camera_roll =  -0.26601432139592862* M_PI / 180.0;
            //  front_camera_yaw = 1.392716491159997* M_PI / 180.0;  

        /*
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
                 */     
        // 定期更新可视化
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

    tf2::Vector3 Transform_point(
        geometry_msgs::msg::TransformStamped transform_stamped,
        tf2::Vector3 point_A,
        const std::string& target_frame, 
        const std::string& source_frame)
    {
        // 使用 tf2::doTransform 来进行坐标变换
        tf2::Transform transform;
        tf2::fromMsg(transform_stamped.transform, transform);  // 从TransformStamped获取tf2::Transform
        tf2::Vector3 point_B = transform * point_A;  // 变换point_A到point_B
    

        // 可视化point_A
        visualization_msgs::msg::Marker marker_point_A;
        marker_point_A.header.frame_id = source_frame;  // 在A坐标系下
        marker_point_A.header.stamp = this->get_clock()->now();
        marker_point_A.ns = "points";
        marker_point_A.id = 0;
        marker_point_A.type = visualization_msgs::msg::Marker::SPHERE;
        marker_point_A.action = visualization_msgs::msg::Marker::ADD;
        marker_point_A.pose.position.x = point_A.x();
        marker_point_A.pose.position.y = point_A.y();
        marker_point_A.pose.position.z = point_A.z();
        marker_point_A.pose.orientation.w = 1.0;  // 默认旋转
    
        marker_point_A.scale.x = 0.2;
        marker_point_A.scale.y = 0.2;
        marker_point_A.scale.z = 0.2;
        marker_point_A.color.r = 1.0f;  // 红色
        marker_point_A.color.g = 0.0f;
        marker_point_A.color.b = 0.0f;
        marker_point_A.color.a = 1.0f;  // 不透明
    
        // 发布 Marker_A
        marker_pub_A->publish(marker_point_A);
    
        // 可视化point_B
        visualization_msgs::msg::Marker marker_point_B;
        marker_point_B.header.frame_id = target_frame;  // 在B坐标系下
        marker_point_B.header.stamp = this->get_clock()->now();
        marker_point_B.ns = "points";
        marker_point_B.id = 1;  // 确保B的ID与A不同
        marker_point_B.type = visualization_msgs::msg::Marker::SPHERE;
        marker_point_B.action = visualization_msgs::msg::Marker::ADD;
        marker_point_B.pose.position.x = point_B.x();
        marker_point_B.pose.position.y = point_B.y();
        marker_point_B.pose.position.z = point_B.z();
        marker_point_B.pose.orientation.w = 1.0;  // 默认旋转
    
        marker_point_B.scale.x = 0.2;
        marker_point_B.scale.y = 0.2;
        marker_point_B.scale.z = 0.2;
        marker_point_B.color.r = 0.0f;  // 绿色
        marker_point_B.color.g = 1.0f;
        marker_point_B.color.b = 0.0f;
        marker_point_B.color.a = 1.0f;  // 不透明
    
        // 发布 Marker_B
        marker_pub_B->publish(marker_point_B);

        return point_B;
    }

    std::vector<cv::Point2f> projectPointToImage(geometry_msgs::msg::TransformStamped transform_stamped, 
                            tf2::Vector3 point_world, 
                            const CameraParams& params)
    {
        // 从 TransformStamped 获取旋转矩阵和平移向量
        tf2::Transform transform;
        tf2::fromMsg(transform_stamped.transform, transform);
        
        tf2::Vector3 point_camera = transform * point_world;  // 世界坐标系点转摄像头坐标系
        

        RCLCPP_INFO(this->get_logger(), "Point in B--: (%.2f, %.2f, %.2f)", point_camera.x(), point_camera.y(), point_camera.z());
        
        // 获取摄像头内参矩阵
        cv::Mat K = (cv::Mat_<double>(3, 3) << 
            params.fx,      0,    params.cx,
            0,         params.fy, params.cy,
            0,              0,        1    );
        
        // 畸变参数
        cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << 
            params.k1, params.k2, params.p1, params.p2, 0.0);  // 这里假设没有高阶畸变
        
        // 旋转矩阵和平移向量
        cv::Mat rvec = (cv::Mat_<double>(3, 1) << 
            0, 0, 0);
        
        cv::Mat tvec = (cv::Mat_<double>(3, 1) << 
            0, 0,0);

        
        // 将3D点转换为2D图像坐标
        std::vector<cv::Point3f> objectPoints = {cv::Point3f(point_camera.x(), point_camera.y(), point_camera.z())};
        //std::vector<cv::Point3f> objectPoints = {cv::Point3f(point_world.x(), point_world.y(), point_world.z())};
        std::vector<cv::Point2f> imagePoints;
        
        // 投影
        cv::projectPoints(objectPoints, rvec, tvec, K, distCoeffs, imagePoints);
        
        // 输出投影后的图像坐标
        if (!imagePoints.empty()) {
            RCLCPP_INFO(this->get_logger(), "Projected point: (%.2f, %.2f)", imagePoints[0].x, imagePoints[0].y);
        }
        return imagePoints;
    }

    void timer_callback()
    {     
        tf2::Vector3 point_world(2.86, 1.2, 0);

        tf2::Vector3 point_camera=Transform_point(transform_stamped,point_world,"front_camera_five",  "base_link");
        // 打印点 A 和 B 的位置
        RCLCPP_INFO(this->get_logger(), "Point in A: (%.2f, %.2f, %.2f)", point_world.x(), point_world.y(), point_world.z());
        RCLCPP_INFO(this->get_logger(), "Point in B: (%.2f, %.2f, %.2f)", point_camera.x(), point_camera.y(), point_camera.z());
            
        
        // 初始化相机内参和畸变参数
        CameraParams params = {
            314.20808654941294,  // fx
            316.37152389536834,  // fy
            639.2087469363039,  // cx
            476.31673181952203,  // cy
            
            -0.10894763660224048, // k1
            0.022162485245748754, // k2
            -0.0074447044434985303, // p1
            0.0016800367866348803  // p2

            // 0.10894763660224048, // k1
            // -0.022162485245748754, // k2
            // 0.0074447044434985303, // p1
            // -0.0016800367866348803  // p2
        };


        std::vector<cv::Point2f> imagePoints=projectPointToImage( transform_stamped, point_world, params);
        publish_image(imagePoints[0].x, imagePoints[0].y,255,0,0);     

        double u,  v;
        projectToPixel(params,  point_camera.x(), point_camera.y(),  point_camera.z(),  u, v);

        publish_image(u, v,0,255,0); 
        
    }


    // 将相机坐标系下的三维点映射到像素坐标系下的函数
    void projectToPixel(const CameraParams& params, double x, double y, double z, double& u, double& v) {
        
        u = (params.fx * x) / z+ params.cx;
        v = (params.fy * y) / z + params.cy; 
        

        // // 计算归一化像素坐标
        // double u_norm = (params.fx * x) / z + params.cx;
        // double v_norm = (params.fy * y) / z + params.cy;
        
        // // 畸变校正
        // double r2 = u_norm * u_norm + v_norm * v_norm;
        // double r4 = r2 * r2;
    
        // double dx = u_norm * (1 + params.k1 * r2 + params.k2 * r4) + 2 * params.p1 * u_norm * v_norm + params.p2 * (r2 + 2 * u_norm * u_norm);
        // double dy = v_norm * (1 + params.k1 * r2 + params.k2 * r4) + 2 * params.p1 * (r2 + 2 * v_norm * v_norm) + params.p2 * u_norm * v_norm;
    
        // // 最终的像素坐标
        // u = dx;
        // v = dy;              
    }
    void publish_image(float x ,float y ,float r ,float b,float g)
    {
        // 加载图像（替换为你自己的图片路径）
        cv::Mat image = cv::imread("/home/parking/hezhenhao/location_test/src/Copy of front_1736825071881.png"); // Copy of 
        
        if (image.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load image!");
            return;
        }

        // 在图像上绘制一个点
        // 设定点的位置为(100, 100)位置，颜色为红色，圆点大小为10
        cv::circle(image, cv::Point(x, y), 5, cv::Scalar(g, b, r), -1);

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

