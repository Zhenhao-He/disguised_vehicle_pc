#include "wgs_convertor.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  auto node = std::make_shared<Wgs_To_Utm_MGRS>("wgs_to_utm_node", node_options);
  rclcpp::spin(node);
  return 0;
}
