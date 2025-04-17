#include <filesystem>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/float32.hpp>
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "lanelet2_projection/UTM.h"
#include <iostream>
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <fstream>
//#include "pc_utm_to_mgrs_converter/pc_utm_to_mgrs_converter.hpp"
#include "msg_interfaces/msg/hc_sentence.hpp"
#include "msg_interfaces/msg/hcinspvatzcb.hpp"
#include "msg_interfaces/msg/hcrawimub.hpp"
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <GeographicLib/MGRS.hpp>
using namespace std;
using namespace Eigen;
class Wgs_To_Utm_MGRS : public rclcpp::Node
{
public:
    enum class MGRSPrecision {
        _1_METER = 5,
        _100MICRO_METER = 9,
    };
    enum class CoordinateSystem {
        UTM = 0,
        MGRS = 1,
    };
    struct GNSSStat
    {

        GNSSStat()
                : coordinate_system(CoordinateSystem::MGRS),
                  northup(true),
                  zone(0),
                  mgrs_zone(""),
                  x(0),
                  y(0),
                  z(0),
                  latitude(0),
                  longitude(0),
                  altitude(0)
        {
        }

        CoordinateSystem coordinate_system;
        bool northup;
        int zone;
        std::string mgrs_zone;
        double x;
        double y;
        double z;
        double latitude;
        double longitude;
        double altitude;
    };
    Wgs_To_Utm_MGRS(const std::string & node_name, const rclcpp::NodeOptions & node_options);
    ~Wgs_To_Utm_MGRS() = default;

private:
    
    void gs_devpvt_callback(const msg_interfaces::msg::Hcinspvatzcb::ConstSharedPtr msg);
    void initialpose_expect_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
    
    geometry_msgs::msg::PoseWithCovarianceStamped wgs2utm(const msg_interfaces::msg::Hcinspvatzcb::ConstSharedPtr msg);
    void  utm2mgrs(const msg_interfaces::msg::Hcinspvatzcb::ConstSharedPtr msg,geometry_msgs::msg::PoseWithCovarianceStamped utm_pose_status, const MGRSPrecision precision, const rclcpp::Logger &logger);

    rclcpp::Subscription<msg_interfaces::msg::Hcinspvatzcb>::SharedPtr gs_devpvt_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_expect_sub_;
    
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gnss_utm_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_mgrs_initialpose;
    rclcpp::Publisher<msg_interfaces::msg::Hcinspvatzcb>::SharedPtr pub_mgrs_devpvt;

    bool first_gps_;
    double lat_ori_;
    double lon_ori_;
    double alt_ori_;
    std::string write_path_raw_;
    std::shared_ptr<lanelet::projection::UtmProjector> utm_projector_;

    geometry_msgs::msg::PoseWithCovarianceStamped utm_pose_status;

    geometry_msgs::msg::PoseWithCovarianceStamped current_mgrs_pose_status;

    struct Pose {
        Vector3d position{0.0, 0.0, 0.0};  // 位置 (x, y, z)
        Quaterniond orientation{0.0, 0.0, 0.0, 1.0};   // 姿态 (四元数 x, y, z, w)
    };
    Pose offset;

};

