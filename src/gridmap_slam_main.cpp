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


#include "rclcpp/rclcpp.hpp"

#include "gridmap_slam/GridMapSlamNode.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  if (argc < 2) {
    std::cerr << "Usage: ros2 run gridmap_slam gridmap_slam_main <map_file.pcd>" << std::endl;
  }

  std::cout << "Using file [" <<  argv[1] << "]" << std::endl;
  auto gridmap_slam_node = gridmap_slam::GridMapSlamNode::make_shared("gridmap_slam", argv[1]);

  rclcpp::spin(gridmap_slam_node);

  rclcpp::shutdown();
  return 0;
}
