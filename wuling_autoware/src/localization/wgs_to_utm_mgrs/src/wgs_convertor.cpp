#include "wgs_convertor.hpp"

Wgs_To_Utm_MGRS::Wgs_To_Utm_MGRS(const std::string & node_name, const rclcpp::NodeOptions & node_options)
: rclcpp::Node(node_name, node_options),
  first_gps_(false)
{
    // std::filesystem::path current_path = std::filesystem::current_path();
    // std::filesystem::path data_folder = current_path / "autoware_data" / "gps_data";
    // if (!std::filesystem::exists(data_folder)) {
    //     if (!std::filesystem::create_directory(data_folder)) {
    //         std::cerr << "Error: Failed to create directory " << data_folder << std::endl;
    //     }
    // }
    // std::filesystem::path new_path = data_folder / "gps_Path_raw.txt";
    // RCLCPP_INFO(this->get_logger(), "Write path: %s", new_path.c_str());


    gs_devpvt_sub_ = this->create_subscription<msg_interfaces::msg::Hcinspvatzcb>(
        "~/input/chcnav/devpvt", 1, std::bind(&Wgs_To_Utm_MGRS::gs_devpvt_callback, this, std::placeholders::_1));
   
    gnss_utm_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("~/output/UTM", 2);
    
    pub_mgrs_devpvt = this->create_publisher<msg_interfaces::msg::Hcinspvatzcb>("~/output/MGRS", 2);


}


void Wgs_To_Utm_MGRS::gs_devpvt_callback(const msg_interfaces::msg::Hcinspvatzcb::ConstSharedPtr msg)
{
    
    if (!first_gps_)
    {
        lat_ori_ = msg->latitude;
        lon_ori_ = msg->longitude;
        alt_ori_ = msg->altitude;

        first_gps_ = true;
        lanelet::GPSPoint gps_zero{lat_ori_, lon_ori_, alt_ori_};
        lanelet::Origin gps_origin(gps_zero);
        utm_projector_ = std::make_shared<lanelet::projection::UtmProjector>(gps_origin, false);
    }
    
     geometry_msgs::msg::PoseWithCovarianceStamped  utm_PoseWithCovarianceStamped=wgs2utm(msg);//发布UTM坐标,并返回
     
     const rclcpp::Logger & logger = this->get_logger();
     utm2mgrs(msg,utm_PoseWithCovarianceStamped,MGRSPrecision::_100MICRO_METER,logger);//发布MGRS坐标,并返回
     

}

geometry_msgs::msg::PoseWithCovarianceStamped Wgs_To_Utm_MGRS::wgs2utm(const msg_interfaces::msg::Hcinspvatzcb::ConstSharedPtr msg)
{
    double gps_lat = msg->latitude;
    double gps_lon = msg->longitude;
    double gps_alt = msg->altitude;

    lanelet::GPSPoint gps_lla{gps_lat, gps_lon, gps_alt};
    lanelet::BasicPoint3d utm_point = utm_projector_->forward(gps_lla);
    
    
   
    double roll = (msg->roll) / 180 * M_PI;
    double pitch = (msg->pitch) / 180 * M_PI;
    double yaw = (msg->yaw) / 180 * M_PI;
    // double offset=90;
    // double new_yaw = (msg->yaw) + offset;
    // while (new_yaw > 180) {
    //     new_yaw -= 360;
    // }
    // while (new_yaw <= -180) {
    //     new_yaw += 360;
    // }
    // double yaw = (new_yaw / 180 * M_PI) ;

    tf2::Quaternion quat;
    quat.setRPY(roll, pitch, yaw);

    geometry_msgs::msg::Quaternion q;
    tf2::convert(quat, q);

    
    utm_pose_status.header.stamp = this->now();
    utm_pose_status.header.frame_id = "map";
    utm_pose_status.pose.pose.position.x = utm_point.x();
    utm_pose_status.pose.pose.position.y = utm_point.y();
    utm_pose_status.pose.pose.position.z = utm_point.z();
    utm_pose_status.pose.pose.orientation = q;
    utm_pose_status.pose.covariance = {0, gps_lat, gps_lon, gps_alt};

    if (msg->stat[0] == 0)
    {
        RCLCPP_INFO(this->get_logger(),"0-初始化");
    }

    else if (msg->stat[0] == 1)//组合惯导RTK浮点
    {
        RCLCPP_INFO(this->get_logger(),"1-卫导模式");
    }
    else if (msg->stat[0] == 2)
    {
        RCLCPP_INFO(this->get_logger(),"2-组合导航模式");
    }
    else if (msg->stat[0] == 3 )
    {
        RCLCPP_INFO(this->get_logger(),"3-纯惯导模式");
    }
    else
    {
     RCLCPP_INFO(this->get_logger(),"未知模式");
    }

        if (msg->stat[1] == 0)
    {
        RCLCPP_INFO(this->get_logger(),"0-不定位不定向 ");
        utm_pose_status.pose.covariance[0] = 0;
    }
    else if (msg->stat[1] == 1)
    {
        RCLCPP_INFO(this->get_logger(),"1-单点定位定向");
        utm_pose_status.pose.covariance[0] = 1;
    }
    else if (msg->stat[1] == 2)
    {
        RCLCPP_INFO(this->get_logger(),"2-伪距差分定位定向");
        utm_pose_status.pose.covariance[0] = 2;
    }
    else if (msg->stat[1] == 3 )
    {
        RCLCPP_INFO(this->get_logger(),"3-组合推算");
        utm_pose_status.pose.covariance[0] = 3;
    }
    else if (msg->stat[1] == 4)//定位无效
    {
        RCLCPP_INFO(this->get_logger(),"4-RTK 稳定解定位定向");
        utm_pose_status.pose.covariance[0] = 4;
    }
    else if (msg->stat[1] == 5)//定位无效
    {
        RCLCPP_INFO(this->get_logger(),"5-RTK浮点解定位定向");
        utm_pose_status.pose.covariance[0] = 5;
    }
    else if (msg->stat[1] == 6)//定位无效
    {
        RCLCPP_INFO(this->get_logger(),"6-单点定位不定向");
        utm_pose_status.pose.covariance[0] = 6;
    }
    else if (msg->stat[1] == 7)//定位无效
    {
        RCLCPP_INFO(this->get_logger(),"7-伪距差分定位不定向");
        utm_pose_status.pose.covariance[0] = 7;
    }

    else if (msg->stat[1] == 8)//定位无效
    {
        RCLCPP_INFO(this->get_logger(),"8-RTK稳定解定位不定向");
        utm_pose_status.pose.covariance[0] = 8;
    }
    else if (msg->stat[1] == 9)//RTK浮点解定位不定向
    {
        RCLCPP_INFO(this->get_logger(),"9-RTK浮点解定位不定向");
        utm_pose_status.pose.covariance[0] = 9;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(),"0-不定位不定向 ");
        utm_pose_status.pose.covariance[0] = 0;  
    }
    /*
    RCLCPP_INFO(rclcpp::get_logger("wgs84 Coordinate:"), "[%.13f %.13f %.13f ]",
                msg->latitude,
                msg->longitude,
                msg->altitude);
    RCLCPP_INFO(rclcpp::get_logger("UTM Coordinate:"), "[%.5f %.5f %.5f ]",
                utm_point.x(),
                utm_point.y(),
                utm_point.z());*/
    utm_pose_status.header.stamp=rclcpp::Clock().now();
    gnss_utm_pub_->publish(utm_pose_status);
    /*
    std::ofstream foutC(new_path, std::ios::app);
    foutC << std::fixed << std::setprecision(12) << utm_pose_status.pose.pose.position.x << " " << utm_pose_status.pose.pose.position.y << std::endl;
    foutC.close();*/
    
    #if 0
    //筛选路径点
    double d_pre2now = (pow(gps_pp_now.pose.position.x - gps_pp_pre.pose.position.x,2)
                        +  pow(gps_pp_now.pose.position.y - gps_pp_pre.pose.position.y,2));
    if(d_pre2now >0.025 || (ros::Time::now().toSec() - pretime) > 0.5) 
    {
        std::ofstream foutC(write_path, ios::app);
        foutC.setf(ios::fixed, ios::floatfield);
        foutC.precision(12);
        foutC <<gps_pp_now.pose.position.x<< " "<<gps_pp_now.pose.position.y<<endl;
        foutC.close();

        gps_pp_pre = gps_pp_now;
        pretime = ros::Time::now().toSec();
        }
        #endif
        return utm_pose_status;
}

void Wgs_To_Utm_MGRS::utm2mgrs(const msg_interfaces::msg::Hcinspvatzcb::ConstSharedPtr msg,geometry_msgs::msg::PoseWithCovarianceStamped utm_pose_status, const MGRSPrecision precision, const rclcpp::Logger &logger){
    int zone = 54;//改
    bool northup = true;
        
    constexpr int GZD_ID_size = 5;  // size of header like "53SPU"
    current_mgrs_pose_status = utm_pose_status;
    try {
        std::string mgrs_code;
        GeographicLib::MGRS::Forward(
                zone, northup, utm_pose_status.pose.pose.position.x, utm_pose_status.pose.pose.position.y, msg->latitude, static_cast<int>(precision), mgrs_code);
        std::string mgrs_zone = std::string(mgrs_code.substr(0, GZD_ID_size));
        //RCLCPP_INFO(rclcpp::get_logger("MGRS"), "mgrs_zone: %s", mgrs_zone.c_str());
        current_mgrs_pose_status.pose.pose.position.x = std::stod(mgrs_code.substr(GZD_ID_size, static_cast<int>(precision))) *
                 std::pow(10, static_cast<int>(MGRSPrecision::_1_METER) -
                             static_cast<int>(precision));  // set unit as [m]
        current_mgrs_pose_status.pose.pose.position.y = std::stod(mgrs_code.substr(
                GZD_ID_size + static_cast<int>(precision), static_cast<int>(precision))) *
                 std::pow(10, static_cast<int>(MGRSPrecision::_1_METER) -
                             static_cast<int>(precision));  // set unit as [m]
                                // set unit as [m]
    } catch (const GeographicLib::GeographicErr & err) {
        RCLCPP_ERROR_STREAM(logger, "Failed to convert from UTM to MGRS" << err.what());
    }
    
    //current_mgrs_pose_status.header.stamp=rclcpp::Clock().now();
    //重新赋值
    msg_interfaces::msg::Hcinspvatzcb devpvt_mgrs_status=*msg;

    //根据RTK天线安装位置做转换,记得换成弧度
    double trans_yaw = (msg->yaw) + 90;
    while (trans_yaw > 180) {
        trans_yaw -= 360;
    }
    while (trans_yaw <= -180) {
        trans_yaw += 360;
    }
    trans_yaw = (trans_yaw / 180 * M_PI) ;
    devpvt_mgrs_status.yaw=trans_yaw;

    devpvt_mgrs_status.latitude=current_mgrs_pose_status.pose.pose.position.x+offset.position[0];
    devpvt_mgrs_status.longitude=current_mgrs_pose_status.pose.pose.position.y+offset.position[1];
    devpvt_mgrs_status.altitude=current_mgrs_pose_status.pose.pose.position.z+offset.position[2];

    pub_mgrs_devpvt->publish(devpvt_mgrs_status);

}
