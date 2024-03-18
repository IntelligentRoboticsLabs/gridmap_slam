// Copyright 2024 Intelligent Robotics Lab
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


#ifndef GRIDMAP_SLAM__GRID_MAP_SLAM_NODE__HPP_
#define GRIDMAP_SLAM__GRID_MAP_SLAM_NODE__HPP_

#include "tf2_ros/transform_listener.h"
#include "octomap/octomap.h"
#include "octomap/Pointcloud.h"
#include "octomap/OcTree.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "tf2_ros/buffer.h"

#include "grid_map_ros/grid_map_ros.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "interactive_markers/interactive_marker_server.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"

namespace gridmap_slam
{

class GridMapSlamNode : public rclcpp::Node
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(GridMapSlamNode)

  GridMapSlamNode(
    const std::string & node_name,
    const std::string & pcd_file,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void map_pc_callback(sensor_msgs::msg::PointCloud2::UniquePtr pc_in);
  void publish_map(const pcl::PointCloud<pcl::PointXYZ> & pc_map);
  void publish_octomap(const octomap::OcTree & octomap);
  void marker_feedback(visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback);

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pc_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_pc_sub_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr map_pc_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> im_server_;
  double resolution_ {0.5};


  /*

  void sensor_pc_callback(sensor_msgs::msg::PointCloud2::UniquePtr pc_in);

  std::unique_ptr<grid_map::GridMap> update_gridmap(const pcl::PointCloud<pcl::PointXYZ> & pc);
  std::unique_ptr<octomap::OcTree> update_octomap(const pcl::PointCloud<pcl::PointXYZ> & pc);
  void publish_octomap(
    const octomap::OcTree & octree, const std_msgs::msg::Header & header,
    rclcpp::Publisher<octomap_msgs::msg::Octomap> & pub);
  void publish_gridmap(const grid_map::GridMap & gridmap, const std_msgs::msg::Header & header);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_pc_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_pc_sub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr gridmap_pub_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr clean_octomap_pub_;

  // Debug
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_pc_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;



  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::unique_ptr<grid_map::GridMap> gridmap_;
  std::unique_ptr<octomap::OcTree> octomap_;
  std::unique_ptr<octomap::OcTree> dynamic_obstacles_;

  double resolution_ {0.5};
  double cleaning_time_ {2.0};
  rclcpp::Time last_cleanning_ts_;
  std::string map_frame_ {"map"};*/
};

}  //  namespace gridmap_slam

#endif  // GRIDMAP_SLAM__GRID_MAP_SLAM_NODE__HPP_
