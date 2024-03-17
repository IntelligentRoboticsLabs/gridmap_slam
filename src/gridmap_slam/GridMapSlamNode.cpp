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


#include "gridmap_slam/GridMapSlamNode.hpp"

#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/common/common.h"
#include "octomap_msgs/conversions.h"
#include "pcl_conversions/pcl_conversions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "pcl_ros/transforms.hpp"
#include "grid_map_ros/grid_map_ros.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_ros/conversions.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"


#include "rclcpp/rclcpp.hpp"


namespace gridmap_slam
{

using std::placeholders::_1;
using namespace std::chrono_literals;

GridMapSlamNode::GridMapSlamNode(
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: Node(node_name, options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  declare_parameter("resolution", resolution_);
  get_parameter("resolution", resolution_);
  declare_parameter("cleaning_time", cleaning_time_);
  get_parameter("cleaning_time", cleaning_time_);
  declare_parameter("map_frame", map_frame_);
  get_parameter("map_frame", map_frame_);

  map_pc_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "input_map", 10, std::bind(&GridMapSlamNode::map_pc_callback, this, _1));
  sensor_pc_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "input_sensor", 1, std::bind(&GridMapSlamNode::sensor_pc_callback, this, _1));

  gridmap_pub_ = create_publisher<grid_map_msgs::msg::GridMap>("grid_map", 10);
  octomap_pub_ = create_publisher<octomap_msgs::msg::Octomap>("octomap", 10);
  clean_octomap_pub_ = create_publisher<octomap_msgs::msg::Octomap>("clean_octomap", 10);

  sensor_pc_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("debug_pc", 10);
  marker_pub_  = create_publisher<visualization_msgs::msg::MarkerArray>("debug_marker", 10);

  last_cleanning_ts_ = now() - rclcpp::Duration::from_seconds(cleaning_time_);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
get_reduced_pc(const sensor_msgs::msg::PointCloud2 & pc, double resolution)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(pc, *pcl_cloud);

  // Reduce the density of the pointcloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);   
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
  voxel_grid_filter.setInputCloud(pcl_cloud);
  voxel_grid_filter.setLeafSize(resolution, resolution, resolution);
  voxel_grid_filter.filter(*filtered_cloud);

  return filtered_cloud;
}

void
GridMapSlamNode::map_pc_callback(sensor_msgs::msg::PointCloud2::UniquePtr pc_in)
{
  auto filtered_cloud = get_reduced_pc(*pc_in, resolution_);

  RCLCPP_INFO(get_logger(), "Map received");
  // update_gridmap(filtered_cloud,pc_in->header);
  // publish_gridmap(pc_in->header);

  octomap_ = update_octomap(*filtered_cloud);
  publish_octomap(*octomap_, pc_in->header, *octomap_pub_);
}

std::tuple<pcl::PointCloud<pcl::PointXYZ>::Ptr, std::string>
transform_cloud(
  const pcl::PointCloud<pcl::PointXYZ> & pc_in, std_msgs::msg::Header & header,
  const std::string & map_frame, tf2_ros::Buffer & tf_buffer)
{
  tf2::Stamped<tf2::Transform> sensor2wf;
  std::string error;

  if (tf_buffer.canTransform(
    map_frame, header.frame_id, tf2_ros::fromMsg(header.stamp), 1s, &error))
  {
    auto sensor2wf_msg = tf_buffer.lookupTransform(
      map_frame, header.frame_id, tf2_ros::fromMsg(header.stamp));

    tf2::fromMsg(sensor2wf_msg, sensor2wf);

    tf2::Vector3 translation = sensor2wf.getOrigin();
    tf2::Quaternion rotation = sensor2wf.getRotation();

    // Convert quaternion to Euler angles
    tf2::Matrix3x3 rotation_matrix(rotation);
    double roll, pitch, yaw;
    rotation_matrix.getRPY(roll, pitch, yaw);

    std::cerr << "Frames " << header.frame_id << " -> " <<  map_frame << std::endl;
    // Print translation
    std::cerr << "Translation: (" << translation.x() << ", " << translation.y() << ", " << translation.z() << ")" << std::endl;

    // Print rotation (Euler angles)
    std::cerr << "Rotation (Euler angles): (Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << ")" << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_ros::transformPointCloud(pc_in, *transformed_cloud, sensor2wf);

    return {transformed_cloud, ""};
  } else {
    return {nullptr, error};
  }
}

std::tuple<octomap::point3d, std::string>
get_sensor_position(
  tf2_ros::Buffer & tf_buffer, const std::string & map_frame,
  const std_msgs::msg::Header & header)
{
  tf2::Stamped<tf2::Transform> map2sensor;
  std::string error;

  if (tf_buffer.canTransform(
    map_frame, header.frame_id, tf2_ros::fromMsg(header.stamp), 1s, &error))
  {
    auto map2sensor_msg = tf_buffer.lookupTransform(
      map_frame, header.frame_id, tf2_ros::fromMsg(header.stamp));

    const float x = static_cast<float>(map2sensor_msg.transform.translation.x);
    const float y = static_cast<float>(map2sensor_msg.transform.translation.y);
    const float z = static_cast<float>(map2sensor_msg.transform.translation.z);

    return {{x, y, z}, ""};
  } else {
    return {octomap::point3d(), error};
  }
}

void
GridMapSlamNode::sensor_pc_callback(sensor_msgs::msg::PointCloud2::UniquePtr pc_in)
{
  if (octomap_ == nullptr) {return;}
  if ((now() - last_cleanning_ts_).seconds() < cleaning_time_) {return;}
  last_cleanning_ts_ = now();

  RCLCPP_INFO(get_logger(), "Cleaning");


  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*pc_in, *pcl_cloud);

  // auto filtered_cloud = get_reduced_pc(*pc_in, resolution_);
  auto [transformed_cloud, error] = transform_cloud(
    *pcl_cloud, pc_in->header, map_frame_, tf_buffer_);

  // Correcto
  // sensor_msgs::msg::PointCloud2 debug_pc;
  // pcl::toROSMsg(*transformed_cloud, debug_pc);
  // debug_pc.header.frame_id = map_frame_;
  // debug_pc.header.stamp = pc_in->header.stamp;
  // sensor_pc_pub_->publish(debug_pc);

  if (transformed_cloud == nullptr) {
    RCLCPP_ERROR(get_logger(), "Error transforming pointcloud %s", error.c_str());
    return;
  }

  // Create if doen't exist
  if (dynamic_obstacles_ == nullptr) {
    dynamic_obstacles_ = std::make_unique<octomap::OcTree>(resolution_);
  }

  auto [oct_start, error2] = get_sensor_position(tf_buffer_, map_frame_, pc_in->header);

  if (error2 != "") {
    RCLCPP_ERROR(get_logger(), "Error getting sensor position %s", error.c_str());
    return;
  }

  std::cerr << "Ray from (" << oct_start.x() << ", " << oct_start.y() << ", " << oct_start.z() << ")" << std::endl;

  const auto & spm = transformed_cloud->points[0];

  visualization_msgs::msg::MarkerArray marker_msg;
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = map_frame_;
  marker.header.stamp = pc_in->header.stamp;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.id = visualization_msgs::msg::Marker::ADD;

  geometry_msgs::msg::Point start;
  start.x = oct_start.x();
  start.y = oct_start.y();
  start.z = oct_start.z();

  geometry_msgs::msg::Point end;
  end.x = spm.x;
  end.y = spm.y;
  end.z = spm.z;

  marker.points = {start, end};

  marker.scale.x = 0.05;
  marker.scale.y = 0.1;
  marker.id = 0;
  marker.color.r = 1.0;
  marker.color.a = 1.0;

  marker_msg.markers.push_back(marker);
  marker_pub_->publish(marker_msg);

  std::cerr << " Point 0 (" << spm.x << ", " << spm.y << ", " << spm.z << ")" << std::endl;

  for (const auto & point : transformed_cloud->points) {

    if (std::isinf(point.x) || std::isnan(point.x)) {continue;}

    octomap::point3d oct_end(point.x, point.x, point.z);
    octomap::KeyRay ray;

    octomap_->computeRayKeys(oct_start, oct_end, ray);

    for (auto it = ray.begin(); it != ray.end(); ++it) {
      octomap::point3d voxel_center = octomap_->keyToCoord(*it);

      if (voxel_center == oct_end) {
        continue;
      }

      octomap::OcTreeNode * node = octomap_->search(voxel_center);
      if (node && octomap_->isNodeOccupied(node)) {
        octomap::OcTreeKey new_voxel_key;
        if (dynamic_obstacles_->coordToKeyChecked(voxel_center, new_voxel_key)) {
            dynamic_obstacles_->updateNode(new_voxel_key, true);
        }
        // break;  // Only one point at a time
      }
    }
  }
//
//    for (auto it = ray.begin(); it != ray.end(); ++it) {  // Exclude last voxel
//      octomap::point3d voxel_center = octomap_->keyToCoord(*it);
//      octomap::OcTreeNode * node = octomap_->search(voxel_center);
//
//      if (node && octomap_->isNodeOccupied(node)) {
//
//        if (!shown) {
//          std::cerr << "Ray from (" << oct_end.x() << ", " << oct_end.y() << ", " << oct_end.z() << ")" << std::endl;
//          shown = true;
//        }
//        // std::cerr << "*";
//        // std::cerr << "(" << voxel_center.x() << ", " << voxel_center.y() << ", " << voxel_center.z() << ")" << std::endl;
//        octomap::OcTreeKey new_voxel_key;
//        if (dynamic_obstacles_->coordToKeyChecked(voxel_center, new_voxel_key)) {
//          dynamic_obstacles_->updateNode(new_voxel_key, true);
//        }
//      }
//    }
//  }
//  std::cerr << std::endl;
//
  std_msgs::msg::Header header;
  header.frame_id = map_frame_;
  header.stamp = pc_in->header.stamp;
  publish_octomap(*dynamic_obstacles_, header, *clean_octomap_pub_);
}

std::unique_ptr<grid_map::GridMap>
GridMapSlamNode::update_gridmap(
  const pcl::PointCloud<pcl::PointXYZ> & pc)
{
  pcl::PointXYZ min_point, max_point;
  pcl::getMinMax3D(pc, min_point, max_point);

  double size_x = max_point.x - min_point.x;
  double size_y = max_point.y - min_point.y;

  auto gridmap = std::make_unique<grid_map::GridMap>();
  gridmap->setFrameId("map");
  gridmap->setGeometry(grid_map::Length(size_x, size_y), resolution_, grid_map::Position(0.0, 0.0));

  gridmap->add("elevation");
  gridmap->add("occupancy");

  for (double x = min_point.x + resolution_; x < max_point.x; x = x + resolution_) {
     for (double y = min_point.y + resolution_; y < max_point.y; y = y + resolution_) {
        grid_map::Position current_pos(x, y);
        gridmap->atPosition("elevation", current_pos) = 0.0;
        gridmap->atPosition("occupancy", current_pos) = 0.0;
     }
  }

  return gridmap;
}

std::unique_ptr<octomap::OcTree>
GridMapSlamNode::update_octomap(
  const pcl::PointCloud<pcl::PointXYZ> & pc)
{
  octomap::Pointcloud octo_cloud;
  for (const auto & point : pc.points) {
    octo_cloud.push_back(point.x, point.y, point.z);
  }

  auto octomap = std::make_unique<octomap::OcTree>(resolution_);
  octomap->insertPointCloud(octo_cloud, octomap::point3d(0, 0, 0));

  return octomap;
}

void
GridMapSlamNode::publish_gridmap(
  const grid_map::GridMap & gridmap, const std_msgs::msg::Header & header)
{
  std::unique_ptr<grid_map_msgs::msg::GridMap> msg;
  msg = grid_map::GridMapRosConverter::toMessage(gridmap);
  msg->header = header;
  gridmap_pub_->publish(std::move(msg));  
}

void
GridMapSlamNode::publish_octomap(
  const octomap::OcTree & octree,const std_msgs::msg::Header & header,
  rclcpp::Publisher<octomap_msgs::msg::Octomap> & pub)
{
  octomap_msgs::msg::Octomap octomap_msg;
  octomap_msgs::binaryMapToMsg(octree, octomap_msg);
  
  octomap_msg.header = header;

  pub.publish(octomap_msg);
}

}  //  namespace gridmap_slam
