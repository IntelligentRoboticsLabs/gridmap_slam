cmake_minimum_required(VERSION 3.8)
project(gridmap_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_BUILD_TYPE RelWithDebInfo)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(std_srvs REQUIRED)

set(dependencies
  rclcpp
  geometry_msgs
  octomap
  octomap_msgs
  octomap_ros
  nav_msgs
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  grid_map_ros
  grid_map_msgs
  grid_map_core
  pcl_conversions
  pcl_ros
  visualization_msgs
  interactive_markers
  std_srvs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/GridMapSlamNode.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(gridmap_slam_main src/gridmap_slam_main.cpp)
ament_target_dependencies(gridmap_slam_main ${dependencies})
target_link_libraries(gridmap_slam_main ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  gridmap_slam_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # find_package(ament_cmake_gtest REQUIRED)
  # add_subdirectory(tests)
endif()

ament_package()
