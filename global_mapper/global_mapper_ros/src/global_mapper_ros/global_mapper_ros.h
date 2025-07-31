// Copyright 2017 Massachusetts Institute of Technology
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <message_filters/subscriber.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_ros/point_cloud.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <snapstack_msgs/State.h>

#include "global_mapper/global_mapper.h"
#include "global_mapper_ros/PlanningGrids.h"

namespace global_mapper_ros
{
class GlobalMapperRos : public rclcpp::Node, public std::enable_shared_from_this<GlobalMapperRos>
{
public:
  GlobalMapperRos() : Node("global_mapper_ros");
  void Run();

private:
  void GetParams();
  void InitImageTransport();
  void InitSubscribers();
  void InitPublishers();
  void PopulateUnknownPointCloudMsg(const voxel_grid::VoxelGrid<float>& occupancy_grid,
                                    sensor_msgs::msg::PointCloud2* pointcloud);
  void PopulateOccupancyPointCloudMsg(const voxel_grid::VoxelGrid<float>& occupancy_grid,
                                      sensor_msgs::msg::PointCloud2* pointcloud);
  void PopulateDistancePointCloudMsg(const voxel_grid::VoxelGrid<int>& distance_grid,
                                     sensor_msgs::msg::PointCloud2* pointcloud);
  void PopulateCostPointCloudMsg(const voxel_grid::VoxelGrid<int>& cost_grid, sensor_msgs::msg::PointCloud2* pointcloud);
  void PopulatePathMsg(const std::vector<std::array<double, 3>>& path, nav_msgs::msg::Path* path_msg);
  void Publish();

  void PublishPlanningGrids(const voxel_grid::VoxelGrid<float>& occupancy_grid,
                            const voxel_grid::VoxelGrid<int>& distance_grid,
                            const voxel_grid::VoxelGrid<int>& cost_grid);

  // callbacks
  void DepthImageCallback(const sensor_msgs::msg::Image::SharedPtr image,
                          const sensor_msgs::msg::CameraInfo::SharedPtr camera_info);
  void PoseCallback(const snapstack_msgs::msg::State::SharedPtr pose_ptr);
  void GoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal_ptr);
  void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_ptr);

  // health and status
  enum ProcessArgs
  {
    NOMINAL = 0,
    NO_POSE = 1,
    NO_GOAL = 2,
    NO_DEPTH_IMAGE = 3
  };

  // name of the drone
  std::string name_drone;

  // publishers
  // ros::Publisher occ_grid_pub_;
  // ros::Publisher unknown_grid_pub_;
  // ros::Publisher frontier_grid_pub_;
  // ros::Publisher dist_grid_pub_;
  // ros::Publisher cost_grid_pub_;
  // ros::Publisher path_pub_;
  // ros::Publisher sparse_path_pub_;
  // ros::Publisher planning_grids_pub_;
  
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occ_grid_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr unknown_grid_pub_;
  // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>SharedPtr frontier_grid_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dist_grid_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cost_grid_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr sparse_path_pub_;
  // rclcpp::Publisher<global_mapper_ros::PlanningGrids>SharedPtr planning_grids_pub_;


  // time (in secs) of the last point cloud fused in this map
  // ros::Time tstampLastPclFused_;
  // ros::Timer grid_pub_timer_;
  rclcpp::Time tstampLastPclFused_;
  rclcpp::TimerBase::SharedPtr grid_pub_timer_;

  // subscribers
  // image_transport::CameraSubscriber depth_sub_;
  // ros::Subscriber pose_sub_;
  // ros::Subscriber odom_sub_;
  // ros::Subscriber goal_sub_;
  image_transport::CameraSubscriber depth_sub_; //unsure of the image transport stuff...
  rclcpp::Subscription<snapstack_msgs::msg::State>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

  // params
  global_mapper::Params params_;
  bool publish_occupancy_grid_;
  bool publish_unknown_grid_;
  bool publish_distance_grid_;
  bool publish_cost_grid_;
  bool publish_path_;
  double clear_unknown_distance_;
  double target_altitude_;

  // i/o flags
  bool got_goal_;
  bool got_pose_;
  bool got_depth_image_;

  std::shared_ptr<global_mapper::GlobalMapper> global_mapper_ptr_;
  // rclcpp::Node::SharedPtr pnh_; //necessary?
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_ptr_;
  std::shared_ptr<image_transport::ImageTransport> it_ptr_;
};
}  // namespace global_mapper_ros
