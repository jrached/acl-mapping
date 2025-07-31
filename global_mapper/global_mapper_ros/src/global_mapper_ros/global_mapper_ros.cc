// Copyright 2017 Massachusetts Institute of Technology

#include <csignal>
#include <memory>
#include <utility>
#include <algorithm>
#include <math.h>

#include <rclcpp/rclcpp.hpp>
#include <pcl_ros/transforms.hpp>
#include <pcl/conversions.hpp>
#include <tf2_sensor_msgs/msg/tf2_sensor_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <builtin_interfaces/msg/Time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <Eigen/Dense>

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <fla_utils/param_utils.hpp>

#include "global_mapper_ros/global_mapper_ros.h"
#include "fla_utils/process_status.hpp"
#include "global_mapper/global_mapper.h"
#include "global_mapper/params.h"

namespace global_mapper_ros
{
GlobalMapperRos::GlobalMapperRos()
  : Node("global_mapper_ros")
  , publish_occupancy_grid_(false)
  , publish_distance_grid_(false)
  , publish_cost_grid_(false)
  , publish_path_(false)
  , clear_unknown_distance_(0.0)
  , target_altitude_(0.0)
{
  // it_ptr_ = std::unique_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(pnh_));
  // tf_listener_ptr_ = std::unique_ptr<tf2_ros::TransformListener>(new tf2_ros::TransformListener(tf_buffer_));
  
  // og code did not define buffer for some reason
  tf_buffer_ptr_= std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ptr_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_ptr_);
  // name_drone = ros::this_node::getNamespace();
  // name_drone.erase(std::remove(name_drone.begin(), name_drone.end(), '/'), name_drone.end());  // remove slashes
  name_drone = this->get_namespace();
  name_drone.erase(std::remove(name_drone.begin(), name_drone.end(), '/'), name_drone.end());  // remove slashes
  
  // std::cout << "The name of the drone is: " << name_drone << std::endl;
}

void GlobalMapperRos::GetParams()
{
  fla_utils::SafeGetParam(this, "global_frame", params_.global_frame);
  fla_utils::SafeGetParam(this, "origin", params_.origin);
  fla_utils::SafeGetParam(this, "world_dimensions", params_.world_dimensions);
  fla_utils::SafeGetParam(this, "resolution", params_.resolution);
  fla_utils::SafeGetParam(this, "radius_drone", params_.radius_drone);
  fla_utils::SafeGetParam(this, "Ra", params_.Ra);
  fla_utils::SafeGetParam(this, "z_ground", params_.z_ground);
  fla_utils::SafeGetParam(this, "skip", params_.skip);
  fla_utils::SafeGetParam(this, "depth_max", params_.depth_max);
  fla_utils::SafeGetParam(this, "r1", params_.r1);
  fla_utils::SafeGetParam(this, "r2", params_.r2);
  fla_utils::SafeGetParam(this, "z_min_unknown", params_.z_min_unknown);
  fla_utils::SafeGetParam(this, "z_max_unknown", params_.z_max_unknown);

  // occupancy_grid
  fla_utils::SafeGetParam(this, "occupancy_grid/init_value", params_.init_value);
  fla_utils::SafeGetParam(this, "occupancy_grid/hit_inc", params_.hit_inc);
  fla_utils::SafeGetParam(this, "occupancy_grid/miss_inc", params_.miss_inc);
  fla_utils::SafeGetParam(this, "occupancy_grid/occupancy_threshold", params_.occupancy_threshold);
  fla_utils::SafeGetParam(this, "occupancy_grid/publish_unknown_grid", publish_unknown_grid_);
  fla_utils::SafeGetParam(this, "occupancy_grid/publish_occupancy_grid", publish_occupancy_grid_);
  fla_utils::SafeGetParam(this, "occupancy_grid/clear_unknown_distance", clear_unknown_distance_);

  // distance_grid
  fla_utils::SafeGetParam(this, "distance_grid/truncation_distance", params_.truncation_distance);
  fla_utils::SafeGetParam(this, "distance_grid/publish_distance_grid", publish_distance_grid_);

  // cost_grid
  fla_utils::SafeGetParam(this, "cost_grid/publish_cost_grid", publish_cost_grid_);
  fla_utils::SafeGetParam(this, "cost_grid/inflation_distance", params_.inflation_distance);
  fla_utils::SafeGetParam(this, "cost_grid/publish_path", publish_path_);
  fla_utils::SafeGetParam(this, "cost_grid/altitude_weight", params_.altitude_weight);
  fla_utils::SafeGetParam(this, "cost_grid/inflation_weight", params_.inflation_weight);
  fla_utils::SafeGetParam(this, "cost_grid/unknown_weight", params_.unknown_weight);
  fla_utils::SafeGetParam(this, "cost_grid/obstacle_weight", params_.obstacle_weight);
  fla_utils::SafeGetParam(this, "cost_grid/target_altitude", target_altitude_);
}

// made new function
void GlobalMapperRos::InitImageTransport() 
{
  it_ptr_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
}

void GlobalMapperRos::InitSubscribers()
{
  // depth_sub_ = it_ptr_->subscribeCamera("depth_image_topic", 1, &GlobalMapperRos::DepthImageCallback, this); maybe??
  // pose_sub_ = pnh_.subscribe("pose_topic", 1, &GlobalMapperRos::PoseCallback, this);
  // goal_sub_ = pnh_.subscribe("goal_topic", 1, &GlobalMapperRos::GoalCallback, this);
  // odom_sub_ = pnh_.subscribe("odom_topic", 1, &GlobalMapperRos::OdomCallback, this);
  depth_sub_ = it_ptr_->subscribeCamera("depth_image_topic", 1, std::bind(&GlobalMapperRos::DepthImageCallback, this, std::placeholders::_1, std::placeholders::_2));
  pose_sub_ = this->create_subscription<snapstack_msgs::msg::State>("pose_topic", 1, std::bind(&GlobalMapperRos::PoseCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom_topic", 1, std::bind(&GlobalMapperRos::OdomCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("goal_topic", 1, std::bind(&GlobalMapperRos::GoalCallback, this, std::placeholders::_1));
}

void GlobalMapperRos::InitPublishers()
{
  if (publish_occupancy_grid_)
  {
    // occ_grid_pub_ = pnh_.advertise<sensor_msgs::PointCloud2>("occupancy_grid_topic", 1);
    occ_grid_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("occupancy_grid_topic", 1);

  }

  if (publish_unknown_grid_)
  {
    // unknown_grid_pub_ = pnh_.advertise<sensor_msgs::PointCloud2>("unknown_grid_topic", 1);
    // frontier_grid_pub_ = pnh_.advertise<sensor_msgs::PointCloud2>("frontier_grid_topic", 1);

    unknown_grid_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("unknown_grid_topic", 1);
    frontier_grid_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("frontier_grid_topic", 1);

  }

  if (publish_distance_grid_)
  {
    // dist_grid_pub_ = pnh_.advertise<sensor_msgs::PointCloud2>("distance_grid_topic", 1);
    dist_grid_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("distance_grid_topic", 1);
   
  }

  if (publish_cost_grid_)
  {
    // cost_grid_pub_ = pnh_.advertise<sensor_msgs::PointCloud2>("cost_grid_topic", 1);
    cost_grid_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cost_grid_topic", 1);
    
  }

  if (publish_path_)
  {
    // path_pub_ = pnh_.advertise<nav_msgs::Path>("path_topic", 1);
    // sparse_path_pub_ = pnh_.advertise<nav_msgs::Path>("sparse_path_topic", 1);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path_topic", 1);
    sparse_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("sparse_path_topic", 1);

  }

  // i dont understand why this was commented out
  // planning_grids_pub_ = pnh_.advertise<global_mapper_ros::PlanningGrids>("planning_grids", 1);

  // grid_pub_timer_ = nh_.createTimer(ros::Duration(0.05), &GlobalMapperRos::Publish, this);
  grid_pub_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50), 
    std::bind(&GlobalMapperRos::Publish, this));
}

void GlobalMapperRos::PopulateUnknownPointCloudMsg(const voxel_grid::VoxelGrid<float>& occupancy_grid,
                                                   sensor_msgs::msg::PointCloud2* pointcloud)
{
  // check for bad input
  if (pointcloud == nullptr)
  {
    return;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  Eigen::Vector3d transform;

  try
  {
    transform_stamped = tf_buffer_.lookupTransform("world", name_drone, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.02));
    transform(0) = transform_stamped.transform.translation.x;
    transform(1) = transform_stamped.transform.translation.y;
    transform(2) = transform_stamped.transform.translation.z;
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_WARN(this->get_logger(), "[world_database_master_ros] OnGetTransform failed with %s", ex.what());

    transform(0) = std::numeric_limits<double>::quiet_NaN();
    transform(1) = std::numeric_limits<double>::quiet_NaN();
    transform(2) = std::numeric_limits<double>::quiet_NaN();
  }

  double xyz[3] = { transform(0), transform(1), transform(2) };
  int slice_ixyz[3];
  occupancy_grid.WorldToGrid(xyz, slice_ixyz);

  int grid_dimensions[3];
  occupancy_grid.GetGridDimensions(grid_dimensions);

  // printf("Ra=%f\n", params_.Ra);
  // If you want all the unknown grid, and cropped to be inside the sphere Sa
  pcl::PointCloud<pcl::PointXYZ> cloud;
  // pcl::PointCloud<pcl::PointXYZ> cloud_frontier;
  double origin[3];
  occupancy_grid.GetOrigin(origin);
  int counter = 0;
  // std::cout << "In PopulateUnknownPointCloudMsg, origin=" << origin[0] << ", " << origin[1] << ", " << origin[2]
  //           << std::endl;
  for (int x = 0; x < grid_dimensions[0]; x = x + 1)
  {
    for (int y = 0; y < grid_dimensions[1]; y = y + 1)
    {
      for (int z = 0; z < grid_dimensions[2]; z = z + 1)
      {
        int ixyz[3] = { x, y, z };
        float occupancy_value = occupancy_grid.ReadValue(ixyz);
        if (global_mapper_ptr_->occupancy_grid_.IsUnknown(occupancy_value))
        {
          occupancy_grid.GridToWorld(ixyz, xyz);
          if (xyz[2] < params_.z_max_unknown && xyz[2] > params_.z_min_unknown)  // only publish points above the ground
          {
            double dist2_to_map_origin =
                pow(xyz[0] - origin[0], 2) + pow(xyz[1] - origin[1], 2) + pow(xyz[2] - origin[2], 2);

            if (sqrt(dist2_to_map_origin) < params_.r2 &&
                sqrt(dist2_to_map_origin) > params_.r1)  // 2 *
                                                         // params_.radius_drone
            {
              cloud.push_back(pcl::PointXYZ(xyz[0], xyz[1], xyz[2]));
            }
          }
        }
        /*        counter = counter + 1;
                if (counter % 5 == 0)  // The frontier grid is downsampled to reduce computational cost
                {
                  // Also let's populate the bounding box point cloud with unknown and free space
                  bool isFrontier = (ixyz[0] == grid_dimensions[0] - 1) || (ixyz[1] == grid_dimensions[1] - 1) ||
                                    (ixyz[2] == grid_dimensions[2] - 1) || ixyz[0] == 0 || ixyz[1] == 0 || ixyz[2] == 0;
                  bool isUnknown = global_mapper_ptr_->occupancy_grid_.IsUnknown(occupancy_value);
                  bool IsOccupied = global_mapper_ptr_->occupancy_grid_.IsOccupied(occupancy_value);
                  bool isFree = (isUnknown == false) && (IsOccupied == false);

                  if (isFrontier && (isFree || isUnknown))
                  {
                    occupancy_grid.GridToWorld(ixyz, xyz);
                    if (xyz[2] > params_.z_ground)  // only publish points above the ground
                    {
                      cloud_frontier.push_back(pcl::PointXYZ(xyz[0], xyz[1], xyz[2]));
                    }
                  }
                }*/
      }
    }
  }

  // If only the slice whit z=z_drone is wanted

  /*  pcl::PointCloud<pcl::PointXYZ> cloud;
    for (int x = 0; x < grid_dimensions[0]; x++)
    {
      for (int y = 0; y < grid_dimensions[1]; y++)
      {
        int ixyz[3] = { x, y, slice_ixyz[2] };
        float occupancy_value = occupancy_grid.ReadValue(ixyz);
        if (global_mapper_ptr_->occupancy_grid_.IsUnknown(occupancy_value))
        {
          occupancy_grid.GridToWorld(ixyz, xyz);
          cloud.push_back(pcl::PointXYZ(xyz[0], xyz[1], xyz[2]));
        }
      }
    }*/

  /*  pcl::toROSMsg(cloud_frontier, *pointcloud_frontier);
    pointcloud_frontier->header.frame_id = "world";
    pointcloud_frontier->header.stamp = tstampLastPclFused_;*/

  pcl::toROSMsg(cloud, *pointcloud);
  pointcloud->header.frame_id = "world"; //use world_frame parameter instead
  pointcloud->header.stamp = tstampLastPclFused_;
}

void GlobalMapperRos::PopulateOccupancyPointCloudMsg(const voxel_grid::VoxelGrid<float>& occupancy_grid,
                                                     sensor_msgs::msg::PointCloud2* pointcloud)
{
  // check for bad input
  if (pointcloud == nullptr)
  {
    return;
  }

  int grid_dimensions[3];
  occupancy_grid.GetGridDimensions(grid_dimensions);

  double xyz[3] = { 0.0 };
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int x = 0; x < grid_dimensions[0]; x++)
  {
    for (int y = 0; y < grid_dimensions[1]; y++)
    {
      for (int z = 0; z < grid_dimensions[2]; z++)
      {
        int ixyz[3] = { x, y, z };
        float occupancy_value = occupancy_grid.ReadValue(ixyz);
        if (global_mapper_ptr_->occupancy_grid_.IsOccupied(occupancy_value))
        {
          occupancy_grid.GridToWorld(ixyz, xyz);
          if (xyz[2] > params_.z_ground)  // only publish points above the ground
          {
            cloud.push_back(pcl::PointXYZ(xyz[0], xyz[1], xyz[2])); //replace with emplace_back (slightly more optimized according to chat)
          }
        }
      }
    }
  }

  pcl::toROSMsg(cloud, *pointcloud);
  pointcloud->header.frame_id = "world";
  // pointcloud->header.stamp = ros::Time::now();
  // I (Jesus) changed the stamp so that it is the same as the last point cloud used in this map
  pointcloud->header.stamp = tstampLastPclFused_;
}

void GlobalMapperRos::PopulateDistancePointCloudMsg(const voxel_grid::VoxelGrid<int>& distance_grid,
                                                    sensor_msgs::msg::PointCloud2* pointcloud)
{
  // check for bad input
  if (pointcloud == nullptr)
  {
    return;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  Eigen::Vector3d transform;

  try
  {
    transform_stamped = tf_buffer_ptr_.lookupTransform("world", name_drone, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.02));
    transform(0) = transform_stamped.transform.translation.x;
    transform(1) = transform_stamped.transform.translation.y;
    transform(2) = transform_stamped.transform.translation.z;
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_WARN(this->get_logger(), "[world_database_master_ros] OnGetTransform failed with %s", ex.what());

    //chat thinks this could be problematic when other functions are called like worldtogrid
    transform(0) = std::numeric_limits<double>::quiet_NaN();
    transform(1) = std::numeric_limits<double>::quiet_NaN();
    transform(2) = std::numeric_limits<double>::quiet_NaN();
  }

  int grid_dimensions[3];
  distance_grid.GetGridDimensions(grid_dimensions);

  double xyz[3] = { transform(0), transform(1), transform(2) };
  int slice_ixyz[3];
  distance_grid.WorldToGrid(xyz, slice_ixyz);

  pcl::PointCloud<pcl::PointXYZRGBA> cloud;
  // chat mentioned preallocated point cloud for better performance with large grids
  static double max_dist = params_.truncation_distance * params_.truncation_distance;
  for (int x = 0; x < grid_dimensions[0]; x++)
  {
    for (int y = 0; y < grid_dimensions[1]; y++)
    {
      int ixyz[3] = { x, y, slice_ixyz[2] };
      distance_grid.GridToWorld(ixyz, xyz);
      int cost = distance_grid.ReadValue(xyz); //chat claims this is used incorrctly and should instead be ReadValue(ixyz)
      pcl::PointXYZRGBA point;
      point.x = xyz[0];
      point.y = xyz[1];
      point.z = xyz[2];
      point.r = static_cast<uint8_t>((max_dist - cost) / max_dist * 255);
      point.g = 0;
      point.b = 0;
      point.a = 255;
      cloud.push_back(point);
    }
  }

  pcl::toROSMsg(cloud, *pointcloud);
  pointcloud->header.frame_id = "world";
  pointcloud->header.stamp = this->now();
}

void GlobalMapperRos::PopulateCostPointCloudMsg(const voxel_grid::VoxelGrid<int>& cost_grid,
                                                sensor_msgs::msg::PointCloud2* pointcloud)
{
  // check for bad input
  if (pointcloud == nullptr)
  {
    return;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  Eigen::Vector3d transform;

  try
  {
    transform_stamped = tf_buffer_ptr_.lookupTransform("world", name_drone, ros::Time(0), ros::Duration(0.02));
    transform(0) = transform_stamped.transform.translation.x;
    transform(1) = transform_stamped.transform.translation.y;
    transform(2) = transform_stamped.transform.translation.z;
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_WARN(this->get_logger(), "[world_database_master_ros] OnGetTransform failed with %s", ex.what());

    transform(0) = std::numeric_limits<double>::quiet_NaN();
    transform(1) = std::numeric_limits<double>::quiet_NaN();
    transform(2) = std::numeric_limits<double>::quiet_NaN();
  }

  int grid_dimensions[3];
  cost_grid.GetGridDimensions(grid_dimensions);

  double xyz[3] = { transform(0), transform(1), transform(2) };
  int slice_ixyz[3];
  cost_grid.WorldToGrid(xyz, slice_ixyz);

  pcl::PointCloud<pcl::PointXYZ> cloud;

  double max_cost = 0;
  double min_cost = std::numeric_limits<double>::max();
  for (int x = 0; x < grid_dimensions[0]; x++)
  {
    for (int y = 0; y < grid_dimensions[1]; y++)
    {
      int ixyz[3] = { x, y, slice_ixyz[2] };
      cost_grid.GridToWorld(ixyz, xyz);
      int cost = cost_grid.ReadValue(xyz);
      if (cost > max_cost && cost != cost_grid::MAX_COST)
      {
        max_cost = cost;
      }
      if (cost < min_cost)
      {
        min_cost = cost;
      }
    }
  }

  for (int x = 0; x < grid_dimensions[0]; x++)
  {
    for (int y = 0; y < grid_dimensions[1]; y++)
    {
      int ixyz[3] = { x, y, slice_ixyz[2] };
      cost_grid.GridToWorld(ixyz, xyz);
      int cost = cost_grid.ReadValue(xyz);
      if (cost == cost_grid::MAX_COST)
      {
        continue;
      }
      pcl::PointXYZ point;
      point.x = xyz[0];
      point.y = xyz[1];
      point.z = (cost - min_cost) / (max_cost - min_cost) * (grid_dimensions[2] >> 1);
      cloud.push_back(point);
    }
  }

  pcl::toROSMsg(cloud, *pointcloud);
  pointcloud->header.frame_id = "world";
  pointcloud->header.stamp = this->now();
}

void GlobalMapperRos::PopulatePathMsg(const std::vector<std::array<double, 3>>& path, nav_msgs::msg::Path* path_msg)
{
  path_msg->header.stamp = this->now();
  path_msg->header.frame_id = "world";
  for (const auto& point : path)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = point[0];
    pose.pose.position.y = point[1];
    pose.pose.position.z = point[2];
    pose.pose.orientation.w = 1.0;
    path_msg->poses.push_back(pose);
  }
}

void GlobalMapperRos::PublishPlanningGrids(const voxel_grid::VoxelGrid<float>& occupancy_grid,
                                           const voxel_grid::VoxelGrid<int>& distance_grid,
                                           const voxel_grid::VoxelGrid<int>& cost_grid)
{
  double origin[3];
  int grid_dimensions[3];
  occupancy_grid.GetOrigin(origin);
  occupancy_grid.GetGridDimensions(grid_dimensions);

  global_mapper_ros::PlanningGrids::Ptr planning_grids_msg(new global_mapper_ros::PlanningGrids);
  // chat generated alternative for creating the message
  // auto planning_grids_msg = std::make_shared<global_mapper_ros::PlanningGrids>();
  planning_grids_msg->header.stamp = this->now();
  planning_grids_msg->header.frame_id = params_.global_frame;
  double projected_goal[3];
  if (!global_mapper_ptr_->GetProjectedGoal(&projected_goal[0]))
  {
    RCLCPP_WARN(this->get_logger(), "Failed to get projected goal")
    return;
  }
  planning_grids_msg->projected_goal[0] = projected_goal[0];
  planning_grids_msg->projected_goal[1] = projected_goal[1];
  planning_grids_msg->projected_goal[2] = projected_goal[2];
  planning_grids_msg->origin[0] = origin[0];
  planning_grids_msg->origin[1] = origin[1];
  planning_grids_msg->origin[2] = origin[2];
  planning_grids_msg->grid_dimensions[0] = grid_dimensions[0];
  planning_grids_msg->grid_dimensions[1] = grid_dimensions[1];
  planning_grids_msg->grid_dimensions[2] = grid_dimensions[2];
  planning_grids_msg->resolution = occupancy_grid.GetResolution();
  planning_grids_msg->occupancy_data = occupancy_grid.GetData();
  planning_grids_msg->distance_data = distance_grid.GetData();
  planning_grids_msg->cost_data = cost_grid.GetData();
  planning_grids_msg->dmax = global_mapper_ptr_->distance_grid_.GetMaxSquaredDistance();
  planning_grids_msg->occupied_threshold = global_mapper_ptr_->occupancy_grid_.GetThreshold();

  planning_grids_pub_.publish(planning_grids_msg);
}

//might be good to add more debug warnings, e.g dense and sparse paths are emtpy
void GlobalMapperRos::Publish()
{
  // get all maps
  voxel_grid::VoxelGrid<float> occupancy_grid;
  voxel_grid::VoxelGrid<int> distance_grid;
  voxel_grid::VoxelGrid<int> cost_grid;

  global_mapper_ptr_->GetVoxelGrids(&occupancy_grid, &distance_grid, &cost_grid);


  // PublishPlanningGrids(occupancy_grid, distance_grid, cost_grid);

  if (publish_occupancy_grid_)
  {
    sensor_msgs::msg::PointCloud2 occ_pointcloud_msg;
    // alternate msg format as suggested by chat
    //auto occ_pointcloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    PopulateOccupancyPointCloudMsg(occupancy_grid, &occ_pointcloud_msg);
    occ_grid_pub_->publish(occ_pointcloud_msg);
  }

  if (publish_unknown_grid_)
  {
    sensor_msgs::msg::PointCloud2 unknown_pointcloud_msg;
    // sensor_msgs::PointCloud2 frontier_pointcloud_msg;
    // PopulateUnknownPointCloudMsg(occupancy_grid, &unknown_pointcloud_msg, &frontier_pointcloud_msg);
    PopulateUnknownPointCloudMsg(occupancy_grid, &unknown_pointcloud_msg);

    unknown_grid_pub_->publish(unknown_pointcloud_msg);
    // frontier_grid_pub_.publish(frontier_pointcloud_msg);
  }

  if (publish_distance_grid_)
  {
    sensor_msgs::msg::PointCloud2 dist_pointcloud_msg;
    PopulateDistancePointCloudMsg(distance_grid, &dist_pointcloud_msg);
    dist_grid_pub_->publish(dist_pointcloud_msg);
  }

  if (publish_cost_grid_)
  {
    sensor_msgs::msg::PointCloud2 cost_pointcloud_msg;
    PopulateCostPointCloudMsg(cost_grid, &cost_pointcloud_msg);
    cost_grid_pub_->publish(cost_pointcloud_msg);
  }

  if (publish_path_)
  {
    double origin_xyz[3];
    global_mapper_ptr_->GetOrigin(origin_xyz);

    std::vector<std::array<double, 3>> dense_path, sparse_path;
    global_mapper_ptr_->GetPaths(&dense_path, &sparse_path);

    nav_msgs::msg::Path dense_path_msg, sparse_path_msg;
    PopulatePathMsg(dense_path, &dense_path_msg);
    PopulatePathMsg(sparse_path, &sparse_path_msg);

    path_pub_->publish(dense_path_msg);
    sparse_path_pub_->publish(sparse_path_msg);
  }
}

// Callback for Odometry (jackal)
void GlobalMapperRos::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_ptr)
{
  // std::cout << "In odom Callback########################" << std::endl;
  RCLCPP_INFO(this->get_logger(), "In OdomCallback ########################");
  double xyz[3] = { odom_ptr->pose.pose.position.x, odom_ptr->pose.pose.position.y, odom_ptr->pose.pose.position.z };
  
  if (!std::isfinite(xyz[0]) || !std::isfinite(xyz[1]) || !std::isfinite(xyz[2])) {
  RCLCPP_WARN(this->get_logger(), "Received invalid odometry position. Skipping update.");
  return;
  }
  
  if (!got_pose_)
  {
    got_pose_ = true;
  }
  global_mapper_ptr_->UpdateOrigin(xyz);
}

void GlobalMapperRos::PoseCallback(const snapstack_msgs::msg::State::SharedPtr pose_ptr)
{
  double xyz[3] = { pose_ptr->pos.x, pose_ptr->pos.y, pose_ptr->pos.z };
  if (!std::isfinite(xyz[0]) || !std::isfinite(xyz[1]) || !std::isfinite(xyz[2])) {
  RCLCPP_WARN(this->get_logger(), "Received invalid pose position. Skipping update.");
  return;
  }
  if (!got_pose_)
  {
    got_pose_ = true;
  }
  
  RCLCPP_DEBUG(this->get_logger(), "Received pose: [%.2f, %.2f, %.2f]", xyz[0], xyz[1], xyz[2]);
  global_mapper_ptr_->UpdateOrigin(xyz);
}

void GlobalMapperRos::GoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal_ptr)
{
  double xyz[3] = { goal_ptr->pose.position.x, goal_ptr->pose.position.y, target_altitude_ };
    if (!std::isfinite(xyz[0]) || !std::isfinite(xyz[1]) || !std::isfinite(xyz[2]))
  {
    RCLCPP_WARN(this->get_logger(), "Received invalid goal coordinates. Skipping.");
    return;
  }
  if (!got_goal_)
  {
    got_goal_ = true;
  }
  RCLCPP_DEBUG(this->get_logger(), "Goal set to: [%.2f, %.2f, %.2f]", xyz[0], xyz[1], xyz[2]);
  global_mapper_ptr_->SetGoal(xyz);
}

void GlobalMapperRos::DepthImageCallback(const sensor_msgs::msg::Image::SharedPtr image_msg,
                                         const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg)
{
  RCLCPP_INFO(this->get_logger(), "Mapper:: DepthImage received");
  // std::cout << image_msg->header.stamp << std::endl;

  if (!got_depth_image_)
  {
    got_depth_image_ = true;
  }
  float cx, cy, fx, fy;
  int binning_x = std::max<uint32_t>(camera_info_msg->binning_x, 1);
  int binning_y = std::max<uint32_t>(camera_info_msg->binning_y, 1);

  fx = camera_info_msg->K[0] / binning_x;
  fy = camera_info_msg->K[4] / binning_y;
  cx = camera_info_msg->K[2] / binning_x;
  cy = camera_info_msg->K[5] / binning_y;

  cv_bridge::CvImageConstPtr depth_ptr = cv_bridge::toCvCopy(image_msg, "32FC1");
  cv::Mat1f depthmap(depth_ptr->image);

  if (image_msg->encoding == "16UC1")
  {
    depthmap /= 1000;  // Convert to meters.
  }

  int height = depthmap.rows;
  int width = depthmap.cols;
  pcl::PointCloud<pcl::PointXYZI> cloud;

  float x_const = 1.0 / fx;
  float y_const = 1.0 / fy;

  // std::cout << "Processing DepthImage" << std::endl;

  // Each pixel can have one of these three values: Finite Number, Nan, Inf.
  // Nan and Inf are NOT finite.

  // TODO: Right now the mapper clears the unknown space when there is a part of the depth image with NaN due to the
  // fact that there is an object is very near the camera. That's why I've put in the asus_camera.urdf.xacro
  // clip/near=0.06 (instead of clip/near>>0 as it was before). But the problem is that I don't know if there is a way
  // to distinguish this case from the case when there is a pixel=NaN that is very far from the camera but that the
  // camera hasn't been able to match it
  for (int i = 0; i < height; i = i + (params_.skip + 1))
  {
    for (int j = 0; j < width; j = j + (params_.skip + 1))
    {
      pcl::PointXYZI point;
      float depth = depthmap(i, j);
      // if(depth<0.001){
      //   depth=std::nan("");
      //  }
      bool finite = std::isfinite(depth);  // False for Nan and Inf
      // bool NaN = (depth != depth);
      bool NaN = (std::isnan(depth));  // True only for finite

      // std::cout<<"I'm Nan= "<<NaN<<std::endl;
      // std::cout<<"Value= "<<depth<<std::endl;
      if (!finite && depth < 0)
      {
        continue;
      }

      if (finite)
      {
        if (depth > params_.depth_max)
        {
          point.z = depth;
          point.intensity = nan("");
        }
        else
        {
          point.z = depth;
          point.intensity = 0;
        }
      }
      else
      {  // Nan and Inf
        point.z = clear_unknown_distance_;
        point.intensity = depth;
      }
      point.x = (j - cx) * point.z * x_const;
      point.y = (i - cy) * point.z * y_const;
      cloud.push_back(point);
    }
  }

  const std::string target_frame = params_.global_frame;
  geometry_msgs::msg::TransformStamped transform_stamped;
  try
  {
    rclcpp::Time stamp(image_msg->header.stamp);
    transform_stamped = tf_buffer_ptr_.lookupTransform(target_frame, image_msg->header.frame_id,
                                                   stamp, rclcpp::Duration::from_seconds(0.12));
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_WARN(this->get_logger(), "[GlobalMapperRos::DepthImageCallback] %s", ex.what());
    return;
  }

  Eigen::Affine3d eigen_transform;
  eigen_transform = tf2::transformToEigen(transform_stamped);

  pcl::PointCloud<pcl::PointXYZI> world_cloud;
  pcl::transformPointCloud(cloud, world_cloud, eigen_transform);

  world_cloud.sensor_origin_ << transform_stamped.transform.translation.x, transform_stamped.transform.translation.y,
      transform_stamped.transform.translation.z, 1;

  global_mapper_ptr_->PushPointCloud(world_cloud.makeShared());

  tstampLastPclFused_ = (*image_msg).header.stamp;
}

void GlobalMapperRos::Run()
{
  GetParams();
  InitSubscribers();
  InitPublishers();
  fla_utils::ProcessStatus process_status(44, 2.0);
  process_status.SetStatus(fla_msgs::msg::ProcessStatus::READY);
  process_status.SetArg(0);

  // start mapping thread
  global_mapper_ptr_ = std::unique_ptr<global_mapper::GlobalMapper>(new global_mapper::GlobalMapper(params_));
  global_mapper_ptr_->Run();

  // handle ros callbacks
  rclcpp::Rate spin_rate(100.0);  // 100 Hz
  while (rclcpp::ok())
  {
    if (!got_pose_)
    {
      process_status.SetStatus(fla_msgs::msg::ProcessStatus::ALARM);
      process_status.SetArg(ProcessArgs::NO_POSE);
    }
    else if (!got_goal_)
    {
      process_status.SetStatus(fla_msgs::msg::ProcessStatus::ALARM);
      process_status.SetArg(ProcessArgs::NO_GOAL);
    }
    else if (!got_depth_image_)
    {
      process_status.SetStatus(fla_msgs::msg::ProcessStatus::ALARM);
      process_status.SetArg(ProcessArgs::NO_DEPTH_IMAGE);
    }
    else
    {
      process_status.SetStatus(fla_msgs::msg::ProcessStatus::READY);
      process_status.SetArg(ProcessArgs::NOMINAL);
    }

    rclcpp::spin_some(shared_from_this());
    spin_rate.sleep();
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<GlobalMapperRos>();
  RCLCPP_INFO(node->get_logger(), "Global Mapper ROS Loop Started.");
  node->Run();

  rclcpp::shutdown();
  return 0;
}


}  // namespace global_mapper_ros
