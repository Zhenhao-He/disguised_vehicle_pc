// Copyright 2022 Tier IV, Inc.
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

#include "autoware/planning_validator/debug_marker.hpp"

#include <autoware/motion_utils/marker/marker_helper.hpp>
#include <autoware/universe_utils/ros/marker_helper.hpp>

#include <memory>
#include <string>
#include <vector>

using visualization_msgs::msg::Marker;

PlanningValidatorDebugMarkerPublisher::PlanningValidatorDebugMarkerPublisher(rclcpp::Node * node)
: node_(node)
{
  debug_viz_pub_ =
    node_->create_publisher<visualization_msgs::msg::MarkerArray>("~/output/marker", 1);

  virtual_wall_pub_ =
    node_->create_publisher<visualization_msgs::msg::MarkerArray>("~/output/virtual_wall", 1);
}

void PlanningValidatorDebugMarkerPublisher::clearMarkers()
{
  marker_array_.markers.clear();
  marker_array_virtual_wall_.markers.clear();
}

void PlanningValidatorDebugMarkerPublisher::pushPoseMarker(
  const autoware_planning_msgs::msg::TrajectoryPoint & p, const std::string & ns, int id)
{
  pushPoseMarker(p.pose, ns, id);
}

void PlanningValidatorDebugMarkerPublisher::pushPoseMarker(
  const geometry_msgs::msg::Pose & pose, const std::string & ns, int id)
{
  using autoware::universe_utils::createMarkerColor;

  // append arrow marker
  std_msgs::msg::ColorRGBA color;
  if (id == 0)  // Red
  {
    color = createMarkerColor(1.0, 0.0, 0.0, 0.999);
  }
  if (id == 1)  // Green
  {
    color = createMarkerColor(0.0, 1.0, 0.0, 0.999);
  }
  if (id == 2)  // Blue
  {
    color = createMarkerColor(0.0, 0.0, 1.0, 0.999);
  }
  Marker marker = autoware::universe_utils::createDefaultMarker(
    "map", node_->get_clock()->now(), ns, getMarkerId(ns), Marker::ARROW,
    autoware::universe_utils::createMarkerScale(0.2, 0.1, 0.3), color);
  marker.lifetime = rclcpp::Duration::from_seconds(0.2);
  marker.pose = pose;

  marker_array_.markers.push_back(marker);
}

void PlanningValidatorDebugMarkerPublisher::pushWarningMsg(
  const geometry_msgs::msg::Pose & pose, const std::string & msg)
{
  visualization_msgs::msg::Marker marker = autoware::universe_utils::createDefaultMarker(
    "map", node_->get_clock()->now(), "warning_msg", 0, Marker::TEXT_VIEW_FACING,
    autoware::universe_utils::createMarkerScale(0.0, 0.0, 1.0),
    autoware::universe_utils::createMarkerColor(1.0, 0.1, 0.1, 0.999));
  marker.lifetime = rclcpp::Duration::from_seconds(0.2);
  marker.pose = pose;
  marker.text = msg;
  marker_array_virtual_wall_.markers.push_back(marker);
}

void PlanningValidatorDebugMarkerPublisher::pushVirtualWall(const geometry_msgs::msg::Pose & pose)
{
  const auto now = node_->get_clock()->now();
  const auto stop_wall_marker = autoware::motion_utils::createStopVirtualWallMarker(
    pose, "autoware_planning_validator", now, 0);
  autoware::universe_utils::appendMarkerArray(stop_wall_marker, &marker_array_virtual_wall_, now);
}

void PlanningValidatorDebugMarkerPublisher::publish()
{
  debug_viz_pub_->publish(marker_array_);
  virtual_wall_pub_->publish(marker_array_virtual_wall_);
}
