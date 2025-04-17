// Copyright 2020-2024 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 *  Copyright (c) 2018, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************/

#ifndef AUTOWARE_COSTMAP_GENERATOR__COSTMAP_GENERATOR_HPP_
#define AUTOWARE_COSTMAP_GENERATOR__COSTMAP_GENERATOR_HPP_

#include "autoware_costmap_generator/objects_to_costmap.hpp"
#include "autoware_costmap_generator/points_to_costmap.hpp"
#include "costmap_generator_node_parameters.hpp"

#include <autoware/universe_utils/system/time_keeper.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tier4_planning_msgs/msg/scenario.hpp>

#include <grid_map_msgs/msg/grid_map.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/static_transform_broadcaster.h"
#include <memory>
#include <string>
#include <vector>
#include <boost/geometry.hpp>
struct costmap_generator_Params {
    std::string costmap_frame;
    std::string vehicle_frame;
    std::string map_frame;
    double update_rate;
    bool activate_by_scenario;
    double grid_min_value;
    double grid_max_value;
    double grid_resolution;
    double grid_length_x;
    double grid_length_y;
    double grid_position_x;
    double grid_position_y;
    double maximum_lidar_height_thres;
    double minimum_lidar_height_thres;
    bool use_wayarea;
    bool use_parkinglot;
    bool use_objects;
    bool use_points;
    double expand_polygon_size;
    int size_of_expansion_kernel;
};

namespace autoware::costmap_generator
{
class CostmapGenerator : public rclcpp::Node
{
public:
  explicit CostmapGenerator(const rclcpp::NodeOptions & node_options);

private:
  costmap_generator_Params param_;

  autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr objects_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr points_;

  grid_map::GridMap costmap_;
  //std::shared_ptr<autoware::universe_utils::TimeKeeper> time_keeper_;

  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_costmap_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_occupancy_grid_;
  rclcpp::Publisher<autoware::universe_utils::ProcessingTimeDetail>::SharedPtr pub_processing_time_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_;
  rclcpp::Subscription<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr sub_objects_;
  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr sub_lanelet_bin_map_;
  rclcpp::Subscription<tier4_planning_msgs::msg::Scenario>::SharedPtr sub_scenario_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;

  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  
  geometry_msgs::msg::PoseStamped::ConstSharedPtr goal_pose_;
  
  std::vector<std::vector<geometry_msgs::msg::Point>> primitives_points_;

  PointsToCostmap points2costmap_;
  ObjectsToCostmap objects2costmap_;

  tier4_planning_msgs::msg::Scenario::ConstSharedPtr scenario_;

  struct LayerName
  {
    static constexpr const char * objects = "objects";
    static constexpr const char * points = "points";
    static constexpr const char * primitives = "primitives";
    static constexpr const char * combined = "combined";
  };

  /// \brief wait for lanelet2 map to load and build routing graph
  void initLaneletMap();

  /// \brief callback for loading lanelet2 map
  void onLaneletMapBin(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg);

  /// \brief callback for DynamicObjectArray
  /// \param[in] in_objects input DynamicObjectArray usually from prediction or perception
  /// component
  void onObjects(const autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg);

  /// \brief callback for sensor_msgs::PointCloud2
  /// \param[in] in_points input sensor_msgs::PointCloud2. Assuming ground-filtered pointcloud
  /// by default
  void onPoints(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  void onScenario(const tier4_planning_msgs::msg::Scenario::ConstSharedPtr msg);
  
  void onGoal(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void onTimer();

  bool isActive();

  /// \brief initialize gridmap parameters based on rosparam
  void initGridmap();

  /// \brief publish ros msg: grid_map::GridMap, and nav_msgs::OccupancyGrid
  /// \param[in] gridmap with calculated cost
  void publishCostmap(const grid_map::GridMap & costmap);





  /// \brief calculate cost from pointcloud data
  /// \param[in] in_points: subscribed pointcloud data
  grid_map::Matrix generatePointsCostmap(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & in_points);

  /// \brief calculate cost from DynamicObjectArray
  /// \param[in] in_objects: subscribed DynamicObjectArray
  grid_map::Matrix generateObjectsCostmap(
    const autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr in_objects);

  /// \brief calculate cost from lanelet2 map
  grid_map::Matrix generatePrimitivesCostmap();

  /// \brief calculate cost for final output
  grid_map::Matrix generateCombinedCostmap();
};
}  // namespace autoware::costmap_generator

#endif  // AUTOWARE_COSTMAP_GENERATOR__COSTMAP_GENERATOR_HPP_
