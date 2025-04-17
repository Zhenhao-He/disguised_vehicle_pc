
/*
using namespace usb_can_ns;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "zlg_can");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");
  usbcan_driver vehcileControl(node, priv_nh);
  ros::Rate loopRate(50);
  while (ros::ok())
  {

    ros::spinOnce();
    loopRate.sleep();
  }

  return 0;
}
*/

#include <signal.h>
#include <rclcpp/rclcpp.hpp>
#include "convert_driver/convert_driver.h"

/*
int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("convert_driver_node");
    auto priv_node = std::make_shared<rclcpp::Node>("priv_convert_driver_node", "", rclcpp::NodeOptions().use_intra_process_comms(true));
     
    usb_can_ns::usbcan_driver vehcileControl(*node, *priv_node);
    rclcpp::executors::MultiThreadedExecutor executor;

    //executor.add_node(node);
    //executor.add_node(priv_node);

    rclcpp::Rate loop_rate(20); // 20Hz

    while (rclcpp::ok())
    {
        
        rclcpp::spin_some(node);
        
        loop_rate.sleep();
    }
    
    rclcpp::shutdown();
    return 0;
}
*/
int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    
    // 创建节点选项
    auto options = rclcpp::NodeOptions().use_intra_process_comms(true);

    // 使用节点选项创建 usbcan_driver 对象
    auto vehcileControl = std::make_shared<usb_can_ns::usbcan_driver>(options);

    // 运行节点
    rclcpp::spin(vehcileControl);

    // 关闭节点
    rclcpp::shutdown();

    return 0;
}
