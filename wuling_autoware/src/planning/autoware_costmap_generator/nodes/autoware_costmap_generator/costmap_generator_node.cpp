#include "autoware_costmap_generator/costmap_generator.hpp"
#include "autoware_costmap_generator/object_map_utils.hpp"

#include <pcl_ros/transforms.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <tf2/time.h>
#include <tf2/utils.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::costmap_generator
{
CostmapGenerator::CostmapGenerator(const rclcpp::NodeOptions & node_options)
: Node("costmap_generator", node_options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{

  RCLCPP_INFO(this->get_logger(), "Initializing CostmapGenerator...");

  param_.costmap_frame = declare_parameter<std::string>("costmap_frame");
  param_.vehicle_frame = declare_parameter<std::string>("vehicle_frame");
  param_.map_frame = declare_parameter<std::string>("map_frame");
  param_.activate_by_scenario = declare_parameter<bool>("activate_by_scenario");
  param_.update_rate = declare_parameter<double>("update_rate");
  param_.grid_min_value = declare_parameter<double>("grid_min_value");
  param_.grid_max_value = declare_parameter<double>("grid_max_value");
  param_.grid_resolution = declare_parameter<double>("grid_resolution");
  param_.grid_length_x = declare_parameter<double>("grid_length_x");
  param_.grid_length_y = declare_parameter<double>("grid_length_y");
  param_.grid_position_x = declare_parameter<double>("grid_position_x");
  param_.grid_position_y = declare_parameter<double>("grid_position_y");
  param_.maximum_lidar_height_thres = declare_parameter<double>("maximum_lidar_height_thres");
  param_.minimum_lidar_height_thres = declare_parameter<double>("minimum_lidar_height_thres");
  param_.use_wayarea = declare_parameter<bool>("use_wayarea");
  param_.use_parkinglot = declare_parameter<bool>("use_parkinglot");
  param_.use_objects = declare_parameter<bool>("use_objects");
  param_.use_points = declare_parameter<bool>("use_points");
  param_.expand_polygon_size = declare_parameter<double>("expand_polygon_size");
  param_.size_of_expansion_kernel = declare_parameter<int>("size_of_expansion_kernel");
  //初始化为世界坐标系原点
  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = get_clock()->now();
  t.header.frame_id = "map";
  t.child_frame_id = "base_link";

  t.transform.translation.x = 0.0;
  t.transform.translation.y = 0.0;
  t.transform.translation.z = 0.0;
  tf2::Quaternion quaternion;
  quaternion.setRPY(0, 0, 0);  // 输入欧拉角：roll, pitch, yaw (单位：弧度)
  t.transform.rotation.x = quaternion.x();
  t.transform.rotation.y = quaternion.y();
  t.transform.rotation.z = quaternion.z();
  t.transform.rotation.w = quaternion.w();

  tf_static_broadcaster_->sendTransform(t);


  // Wait for first tf
  // We want to do this before creating subscriptions
  while (rclcpp::ok()) {
    try {
      tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
      break;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "waiting for initial pose...");
    }
    rclcpp::sleep_for(std::chrono::milliseconds(5000));
  }

  using std::placeholders::_1;
  /*
  sub_objects_ = this->create_subscription<autoware_perception_msgs::msg::PredictedObjects>(
    "~/input/objects", 1, std::bind(&CostmapGenerator::onObjects, this, _1));
  sub_points_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/points_no_ground", rclcpp::SensorDataQoS(),
    std::bind(&CostmapGenerator::onPoints, this, _1));
  sub_lanelet_bin_map_ = this->create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&CostmapGenerator::onLaneletMapBin, this, _1));
  sub_scenario_ = this->create_subscription<tier4_planning_msgs::msg::Scenario>(
    "~/input/scenario", 1, std::bind(&CostmapGenerator::onScenario, this, _1));
  */

  pub_costmap_ = this->create_publisher<grid_map_msgs::msg::GridMap>("~/output/grid_map", 1);
  pub_occupancy_grid_ =
    this->create_publisher<nav_msgs::msg::OccupancyGrid>("~/output/occupancy_grid", 1);
  pub_processing_time_ =
    create_publisher<autoware::universe_utils::ProcessingTimeDetail>("processing_time", 1);
  //time_keeper_ = std::make_shared<autoware::universe_utils::TimeKeeper>(pub_processing_time_);

  const auto period_ns = rclcpp::Rate(param_.update_rate).period();
  timer_ =
    rclcpp::create_timer(this, get_clock(), period_ns, std::bind(&CostmapGenerator::onTimer, this));

  initGridmap();
  
}


void CostmapGenerator::onTimer()
{

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf =
      tf_buffer_.lookupTransform(param_.costmap_frame, param_.vehicle_frame, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    return;
  }
  //time_keeper_->end_track("lookupTransform");

  // Set grid center
  grid_map::Position p;
  p.x() = tf.transform.translation.x;
  p.y() = tf.transform.translation.y;
  costmap_.setPosition(p);

  /*
  if ((param_.use_wayarea || param_.use_parkinglot) && lanelet_map_) {
  
    costmap_[LayerName::primitives] = generatePrimitivesCostmap();
  }

  if (param_.use_objects && objects_) {

    costmap_[LayerName::objects] = generateObjectsCostmap(objects_);
  }

  if (param_.use_points && points_) {

    costmap_[LayerName::points] = generatePointsCostmap(points_);
  }
*/
  {
    costmap_[LayerName::combined] = generateCombinedCostmap();
  }
  
  publishCostmap(costmap_);
}



void CostmapGenerator::initGridmap()
{
  costmap_.setFrameId(param_.costmap_frame);
  costmap_.setGeometry(
    grid_map::Length(param_.grid_length_x, param_.grid_length_y), param_.grid_resolution,
    grid_map::Position(param_.grid_position_x, param_.grid_position_y));

  costmap_.add(LayerName::points, param_.grid_min_value);
  costmap_.add(LayerName::objects, param_.grid_min_value);
  costmap_.add(LayerName::primitives, param_.grid_min_value);
  costmap_.add(LayerName::combined, param_.grid_min_value);
}
grid_map::Matrix CostmapGenerator::generateCombinedCostmap()
{
  // assuming combined_costmap is calculated by element wise max operation
  grid_map::GridMap combined_costmap = costmap_;

  combined_costmap[LayerName::combined].setConstant(param_.grid_min_value);

  combined_costmap[LayerName::combined] =
    combined_costmap[LayerName::combined].cwiseMax(combined_costmap[LayerName::points]);

  combined_costmap[LayerName::combined] =
    combined_costmap[LayerName::combined].cwiseMax(combined_costmap[LayerName::primitives]);

  combined_costmap[LayerName::combined] =
    combined_costmap[LayerName::combined].cwiseMax(combined_costmap[LayerName::objects]);

  return combined_costmap[LayerName::combined];
}
void CostmapGenerator::publishCostmap(const grid_map::GridMap & costmap)
{
  // Set header
  std_msgs::msg::Header header;
  header.frame_id = param_.costmap_frame;
  header.stamp = this->now();

  // Publish OccupancyGrid
  nav_msgs::msg::OccupancyGrid out_occupancy_grid;
  grid_map::GridMapRosConverter::toOccupancyGrid(
    costmap, LayerName::combined, param_.grid_min_value, param_.grid_max_value,
    out_occupancy_grid);
  out_occupancy_grid.header = header;
  pub_occupancy_grid_->publish(out_occupancy_grid);

  // Publish GridMap
  auto out_gridmap_msg = grid_map::GridMapRosConverter::toMessage(costmap);
  out_gridmap_msg->header = header;
  pub_costmap_->publish(*out_gridmap_msg);
}



}  // namespace autoware::costmap_generator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::costmap_generator::CostmapGenerator)
