// Copyright 2017 Massachusetts Institute of Technology
#include "global_mapper_ros/global_mapper_ros.h"

using namespace std::chrono_literals;

namespace global_mapper_ros
{
  GlobalMapperRos::GlobalMapperRos()
      : Node("global_mapper_ros"), start_time_(this->now().seconds()), cloud_(new pcl::PointCloud<pcl::PointXYZ>)
  {

    // Define transform buffer and listener 
    tf_buffer_ptr_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ptr_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_ptr_);
    name_drone_ = this->get_namespace();
    name_drone_.erase(std::remove(name_drone_.begin(), name_drone_.end(), '/'), name_drone_.end()); // remove slashes

    // Instantiate cloud pointer to empty cloud message 
    const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();

  }

  void GlobalMapperRos::GetParams()
  {

    // --- declare all parameters with sensible defaults:
    this->declare_parameter<std::string>("global_frame", "map");
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("sensor_frame", "base_link");
    this->declare_parameter<std::vector<double>>("origin", {1.53, -3.17, 0.82});
    this->declare_parameter<std::vector<double>>("world_dimensions", {16.0, 16.0, 10.0});
    this->declare_parameter<double>("resolution", 0.4);
    this->declare_parameter<double>("radius_drone", 0.1);
    this->declare_parameter<double>("z_ground", 0.1);
    this->declare_parameter<int>("skip", 0);
    this->declare_parameter<double>("depth_max", 8.0);
    this->declare_parameter<double>("r1", 0.8);
    this->declare_parameter<double>("r2", 8.0);
    this->declare_parameter<double>("z_min_unknown", 0.1);
    this->declare_parameter<double>("z_max_unknown", 5.0);
    this->declare_parameter<double>("cloud_ds_size", 0.1);
    this->declare_parameter<bool>("verbose", false); 
    this->declare_parameter<bool>("downsample", true); 

    // namespaced ones:
    this->declare_parameter<double>("occupancy_grid.init_value", 0.0);
    this->declare_parameter<double>("occupancy_grid.hit_inc", 0.4);
    this->declare_parameter<double>("occupancy_grid.miss_inc", -0.4);
    this->declare_parameter<double>("occupancy_grid.occupancy_threshold", 0.6);
    this->declare_parameter<bool>("occupancy_grid.publish_occupancy_grid", true);
    this->declare_parameter<bool>("occupancy_grid.publish_unknown_grid", true);
    this->declare_parameter<double>("occupancy_grid.clear_unknown_distance", 5.0);

    this->declare_parameter<int>("distance_grid.truncation_distance", 6);
    this->declare_parameter<bool>("distance_grid.publish_distance_grid", false);

    this->declare_parameter<bool>("cost_grid.publish_cost_grid", false);
    this->declare_parameter<bool>("cost_grid.publish_path", false);
    this->declare_parameter<int>("cost_grid.inflation_distance", 4);
    this->declare_parameter<int>("cost_grid.altitude_weight", 20);
    this->declare_parameter<int>("cost_grid.inflation_weight", 0);
    this->declare_parameter<int>("cost_grid.unknown_weight", 20);
    this->declare_parameter<int>("cost_grid.obstacle_weight", 10000);
    this->declare_parameter<double>("cost_grid.target_altitude", 2.0);
    
    this->declare_parameter<bool>("temporal_grid.publish_dynamic_grid", true);
    this->declare_parameter<bool>("temporal_grid.publish_static_grid", false);
    this->declare_parameter<double>("temporal_grid.occupied_thresh", 3.0); 
    this->declare_parameter<double>("temporal_grid.unoccupied_thresh", 0.5); 
    this->declare_parameter<int>("temporal_grid.neighbor_radius", 1); 
    this->declare_parameter<int>("temporal_grid.static_neighbor_thresh", 1);

    fla_utils::SafeGetParam(*this, "global_frame", params_.global_frame);
    fla_utils::SafeGetParam(*this, "odom_frame", params_.odom_frame);
    fla_utils::SafeGetParam(*this, "sensor_frame", params_.sensor_frame);
    fla_utils::SafeGetParam(*this, "origin", params_.origin);
    fla_utils::SafeGetParam(*this, "world_dimensions", params_.world_dimensions);
    fla_utils::SafeGetParam(*this, "resolution", params_.resolution);
    fla_utils::SafeGetParam(*this, "radius_drone", params_.radius_drone);
    fla_utils::SafeGetParam(*this, "z_ground", params_.z_ground);
    fla_utils::SafeGetParam(*this, "skip", params_.skip);
    fla_utils::SafeGetParam(*this, "depth_max", params_.depth_max);
    fla_utils::SafeGetParam(*this, "r1", params_.r1);
    fla_utils::SafeGetParam(*this, "r2", params_.r2);
    fla_utils::SafeGetParam(*this, "z_min_unknown", params_.z_min_unknown);
    fla_utils::SafeGetParam(*this, "z_max_unknown", params_.z_max_unknown);
    fla_utils::SafeGetParam(*this, "cloud_ds_size", params_.cloud_ds_size);
    fla_utils::SafeGetParam(*this, "verbose", params_.verbose); 
    fla_utils::SafeGetParam(*this, "downsample", params_.downsample); 

    // occupancy_grid
    fla_utils::SafeGetParam(*this, "occupancy_grid.init_value", params_.init_value);
    fla_utils::SafeGetParam(*this, "occupancy_grid.hit_inc", params_.hit_inc);
    fla_utils::SafeGetParam(*this, "occupancy_grid.miss_inc", params_.miss_inc);
    fla_utils::SafeGetParam(*this, "occupancy_grid.occupancy_threshold", params_.occupancy_threshold);
    fla_utils::SafeGetParam(*this, "occupancy_grid.publish_unknown_grid", params_.publish_unknown_grid);
    fla_utils::SafeGetParam(*this, "occupancy_grid.publish_occupancy_grid", params_.publish_occupancy_grid);
    fla_utils::SafeGetParam(*this, "occupancy_grid.clear_unknown_distance", params_.clear_unknown_distance);

    // distance_grid
    fla_utils::SafeGetParam(*this, "distance_grid.truncation_distance", params_.truncation_distance);
    fla_utils::SafeGetParam(*this, "distance_grid.publish_distance_grid", params_.publish_distance_grid);

    // cost_grid
    fla_utils::SafeGetParam(*this, "cost_grid.publish_cost_grid", params_.publish_cost_grid);
    fla_utils::SafeGetParam(*this, "cost_grid.inflation_distance", params_.inflation_distance);
    fla_utils::SafeGetParam(*this, "cost_grid.publish_path", params_.publish_path);
    fla_utils::SafeGetParam(*this, "cost_grid.altitude_weight", params_.altitude_weight);
    fla_utils::SafeGetParam(*this, "cost_grid.inflation_weight", params_.inflation_weight);
    fla_utils::SafeGetParam(*this, "cost_grid.unknown_weight", params_.unknown_weight);
    fla_utils::SafeGetParam(*this, "cost_grid.obstacle_weight", params_.obstacle_weight);
    fla_utils::SafeGetParam(*this, "cost_grid.target_altitude", params_.target_altitude);

    // temporal_grid 
    fla_utils::SafeGetParam(*this, "temporal_grid.publish_dynamic_grid", params_.publish_dynamic_grid);
    fla_utils::SafeGetParam(*this, "temporal_grid.publish_static_grid", params_.publish_static_grid);
    fla_utils::SafeGetParam(*this, "temporal_grid.occupied_thresh", params_.occupied_thresh);
    fla_utils::SafeGetParam(*this, "temporal_grid.unoccupied_thresh", params_.unoccupied_thresh); 
    fla_utils::SafeGetParam(*this, "temporal_grid.neighbor_radius", params_.neighbor_radius); 
    fla_utils::SafeGetParam(*this, "temporal_grid.static_neighbor_thresh", params_.static_neighbor_thresh);

    // Print the parameters to the console
    RCLCPP_INFO(this->get_logger(), "Global Mapper Parameters:");
    RCLCPP_INFO(this->get_logger(), "  global_frame: %s", params_.global_frame.c_str());
    RCLCPP_INFO(this->get_logger(), "  origin: [%f, %f, %f]", params_.origin[0], params_.origin[1], params_.origin[2]);
    RCLCPP_INFO(this->get_logger(), "  world_dimensions: [%f, %f, %f]", params_.world_dimensions[0], params_.world_dimensions[1], params_.world_dimensions[2]);
    RCLCPP_INFO(this->get_logger(), "  resolution: %f", params_.resolution);
    RCLCPP_INFO(this->get_logger(), "  radius_drone: %f", params_.radius_drone);
    RCLCPP_INFO(this->get_logger(), "  z_ground: %f", params_.z_ground);
    RCLCPP_INFO(this->get_logger(), "  skip: %d", params_.skip);
    RCLCPP_INFO(this->get_logger(), "  depth_max: %f", params_.depth_max);
    RCLCPP_INFO(this->get_logger(), "  r1: %f", params_.r1);
    RCLCPP_INFO(this->get_logger(), "  r2: %f", params_.r2);
    RCLCPP_INFO(this->get_logger(), "  z_min_unknown: %f", params_.z_min_unknown);
    RCLCPP_INFO(this->get_logger(), "  z_max_unknown: %f", params_.z_max_unknown);
    RCLCPP_INFO(this->get_logger(), "  cloud voxel grid downsample leaf size: %f", params_.cloud_ds_size);
    RCLCPP_INFO(this->get_logger(), "  occupancy_grid.init_value: %f", params_.init_value);
    RCLCPP_INFO(this->get_logger(), "  occupancy_grid.hit_inc: %f", params_.hit_inc);
    RCLCPP_INFO(this->get_logger(), "  occupancy_grid.miss_inc: %f", params_.miss_inc);
    RCLCPP_INFO(this->get_logger(), "  occupancy_grid.occupancy_threshold: %f", params_.occupancy_threshold);
    RCLCPP_INFO(this->get_logger(), "  occupancy_grid.publish_unknown_grid: %s", params_.publish_unknown_grid ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  occupancy_grid.publish_occupancy_grid: %s", params_.publish_occupancy_grid ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  occupancy_grid.clear_unknown_distance: %f", params_.clear_unknown_distance);
    RCLCPP_INFO(this->get_logger(), "  distance_grid.truncation_distance: %d", params_.truncation_distance);
    RCLCPP_INFO(this->get_logger(), "  distance_grid.publish_distance_grid: %s", params_.publish_distance_grid ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  cost_grid.publish_cost_grid: %s", params_.publish_cost_grid ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  cost_grid.inflation_distance: %d", params_.inflation_distance);
    RCLCPP_INFO(this->get_logger(), "  cost_grid.publish_path: %s", params_.publish_path ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  cost_grid.altitude_weight: %d", params_.altitude_weight);
    RCLCPP_INFO(this->get_logger(), "  cost_grid.inflation_weight: %d", params_.inflation_weight);
    RCLCPP_INFO(this->get_logger(), "  cost_grid.unknown_weight: %d", params_.unknown_weight);
    RCLCPP_INFO(this->get_logger(), "  cost_grid.obstacle_weight: %d", params_.obstacle_weight);
    RCLCPP_INFO(this->get_logger(), "  cost_grid.target_altitude: %f", params_.target_altitude);
    RCLCPP_INFO(this->get_logger(), "  temporal_grid.publish_dynamic_grid: %s", params_.publish_dynamic_grid ? "true" : "false");
  }

  void GlobalMapperRos::InitSubscribers()
  {
    rclcpp::QoS odom_qos(rclcpp::KeepLast(1));
    odom_qos.best_effort().durability_volatile();

    rclcpp::QoS cloud_qos(rclcpp::KeepLast(1));
    cloud_qos.reliable();

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("pose_topic", odom_qos, std::bind(&GlobalMapperRos::PoseCallback, this, std::placeholders::_1));
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("goal_topic", 1, std::bind(&GlobalMapperRos::GoalCallback, this, std::placeholders::_1));
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("depth_pointcloud_topic", cloud_qos, std::bind(&GlobalMapperRos::PointCloudCallback, this, std::placeholders::_1));
  }

  void GlobalMapperRos::InitPublishers()
  {

    rclcpp::QoS sensor_qos(rclcpp::KeepLast(1));
    sensor_qos.best_effort().durability_volatile();

    rclcpp::QoS cloud_qos(rclcpp::KeepLast(1));
    cloud_qos.reliable();

    if (params_.publish_occupancy_grid)
    {
      occ_grid_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("occupancy_grid_topic", sensor_qos);
    }

    if (params_.publish_unknown_grid)
    {
      unknown_grid_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("unknown_grid_topic", sensor_qos);
      frontier_grid_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("frontier_grid_topic", sensor_qos);
    }

    if (params_.publish_distance_grid)
    {
      dist_grid_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("distance_grid_topic", sensor_qos);
    }

    if (params_.publish_cost_grid)
    {
      cost_grid_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cost_grid_topic", sensor_qos);
    }

    if (params_.publish_path)
    {
      path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path_topic", sensor_qos);
      sparse_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("sparse_path_topic", sensor_qos);
    }

    if (params_.publish_dynamic_grid)
    {
      dynamic_grid_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("dynamic_grid_topic", cloud_qos);
    }

    if (params_.publish_static_grid)
    {
      static_grid_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("static_grid_topic", sensor_qos);
    }

    grid_pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&GlobalMapperRos::Publish, this));
  }

  void GlobalMapperRos::PopulateUnknownPointCloudMsg(const voxel_grid::VoxelGrid<float> &occupancy_grid,
                                                     sensor_msgs::msg::PointCloud2 *pointcloud)
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
      transform_stamped = tf_buffer_ptr_->lookupTransform(params_.global_frame, params_.odom_frame, rclcpp::Time(0), 20ms);
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

    double xyz[3] = {transform(0), transform(1), transform(2)};
    int slice_ixyz[3];
    occupancy_grid.WorldToGrid(xyz, slice_ixyz);

    int grid_dimensions[3];
    occupancy_grid.GetGridDimensions(grid_dimensions);

    // If you want all the unknown grid, and cropped to be inside the sphere Sa
    pcl::PointCloud<pcl::PointXYZ> cloud;
    double origin[3];
    occupancy_grid.GetOrigin(origin);
    int counter = 0;
    for (int x = 0; x < grid_dimensions[0]; x = x + 1)
    {
      for (int y = 0; y < grid_dimensions[1]; y = y + 1)
      {
        for (int z = 0; z < grid_dimensions[2]; z = z + 1)
        {
          int ixyz[3] = {x, y, z};
          float occupancy_value = occupancy_grid.ReadValue(ixyz);
          if (global_mapper_ptr_->occupancy_grid_.IsUnknown(occupancy_value))
          {
            occupancy_grid.GridToWorld(ixyz, xyz);
            if (xyz[2] < params_.z_max_unknown && xyz[2] > params_.z_min_unknown) // only publish points above the ground
            {
              double dist2_to_map_origin =
                  pow(xyz[0] - origin[0], 2) + pow(xyz[1] - origin[1], 2) + pow(xyz[2] - origin[2], 2);

              if (sqrt(dist2_to_map_origin) < params_.r2 &&
                  sqrt(dist2_to_map_origin) > params_.r1) // 2 *
                                                          // params_.radius_drone
              {
                cloud.push_back(pcl::PointXYZ(xyz[0], xyz[1], xyz[2]));
              }
            }
          }
        }
      }
    }

    pcl::toROSMsg(cloud, *pointcloud);
    pointcloud->header.frame_id = params_.global_frame; // use world_frame parameter instead
    pointcloud->header.stamp = pc_stamp_;
  }

  void GlobalMapperRos::PopulateOccupancyPointCloudMsg(const voxel_grid::VoxelGrid<float> &occupancy_grid,
                                                       sensor_msgs::msg::PointCloud2 *pointcloud)
  {
    // check for bad input
    if (pointcloud == nullptr)
    {
      return;
    }

    int grid_dimensions[3];
    occupancy_grid.GetGridDimensions(grid_dimensions);

    double xyz[3] = {0.0};
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (int x = 0; x < grid_dimensions[0]; x++)
    {
      for (int y = 0; y < grid_dimensions[1]; y++)
      {
        for (int z = 0; z < grid_dimensions[2]; z++)
        {
          int ixyz[3] = {x, y, z};
          float occupancy_value = occupancy_grid.ReadValue(ixyz);
          if (global_mapper_ptr_->occupancy_grid_.IsOccupied(occupancy_value))
          {
            occupancy_grid.GridToWorld(ixyz, xyz);
            if (xyz[2] > params_.z_ground) // only publish points above the ground
            {
              cloud.push_back(pcl::PointXYZ(xyz[0], xyz[1], xyz[2])); 
            }
          }
        }
      }
    }
    
    pcl::toROSMsg(cloud, *pointcloud);
    pointcloud->header.frame_id = params_.global_frame;
    pointcloud->header.stamp = pc_stamp_;
  }

  void GlobalMapperRos::PopulateDistancePointCloudMsg(const voxel_grid::VoxelGrid<int> &distance_grid,
                                                      sensor_msgs::msg::PointCloud2 *pointcloud)
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
      transform_stamped = tf_buffer_ptr_->lookupTransform(params_.global_frame, params_.odom_frame, rclcpp::Time(0), 20ms);
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
    distance_grid.GetGridDimensions(grid_dimensions);

    double xyz[3] = {transform(0), transform(1), transform(2)};
    int slice_ixyz[3];
    distance_grid.WorldToGrid(xyz, slice_ixyz);

    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    static double max_dist = params_.truncation_distance * params_.truncation_distance;
    for (int x = 0; x < grid_dimensions[0]; x++)
    {
      for (int y = 0; y < grid_dimensions[1]; y++)
      {
        int ixyz[3] = {x, y, slice_ixyz[2]};
        distance_grid.GridToWorld(ixyz, xyz);
        int cost = distance_grid.ReadValue(xyz); 
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
    pointcloud->header.frame_id = params_.global_frame;
    pointcloud->header.stamp = this->now();
  }

  void GlobalMapperRos::PopulateCostPointCloudMsg(const voxel_grid::VoxelGrid<int> &cost_grid,
                                                  sensor_msgs::msg::PointCloud2 *pointcloud)
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
      transform_stamped = tf_buffer_ptr_->lookupTransform(params_.global_frame, params_.odom_frame, rclcpp::Time(0), 20ms);
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

    double xyz[3] = {transform(0), transform(1), transform(2)};
    int slice_ixyz[3];
    cost_grid.WorldToGrid(xyz, slice_ixyz);

    pcl::PointCloud<pcl::PointXYZ> cloud;

    double max_cost = 0;
    double min_cost = std::numeric_limits<double>::max();
    for (int x = 0; x < grid_dimensions[0]; x++)
    {
      for (int y = 0; y < grid_dimensions[1]; y++)
      {
        int ixyz[3] = {x, y, slice_ixyz[2]};
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
        int ixyz[3] = {x, y, slice_ixyz[2]};
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
    pointcloud->header.frame_id = params_.global_frame;
    pointcloud->header.stamp = this->now();
  }

  void GlobalMapperRos::PopulatePathMsg(const std::vector<std::array<double, 3>> &path, nav_msgs::msg::Path *path_msg)
  {
    path_msg->header.stamp = this->now();
    path_msg->header.frame_id = params_.global_frame;
    for (const auto &point : path)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = point[0];
      pose.pose.position.y = point[1];
      pose.pose.position.z = point[2];
      pose.pose.orientation.w = 1.0;
      path_msg->poses.push_back(pose);
    }
  }

 void GlobalMapperRos::PopulateDynamicPointCloudMsg(const voxel_grid::VoxelGrid<float>& occupancy_grid, 
                                                     const voxel_grid::VoxelGrid<std::vector<double>>& temporal_grid, 
                                                     sensor_msgs::msg::PointCloud2* dynamic_pointcloud,
                                                     sensor_msgs::msg::PointCloud2* static_pointcloud)
{
     // check for bad input
    if (dynamic_pointcloud == nullptr || static_pointcloud == nullptr)
    {
      return;
    }

    // Declare clouds to be populated 
    pcl::PointCloud<pcl::PointXYZ> dynamic_cloud;
    pcl::PointCloud<pcl::PointXYZ> static_cloud;

    #pragma omp parallel
    { 
      double xyz[3] = {0.0};
      int ixyz[3] = {0};

      pcl::PointCloud<pcl::PointXYZ> local_dynamic_cloud; 
      pcl::PointCloud<pcl::PointXYZ> local_static_cloud; 
      #pragma omp for nowait 
      for (size_t i = 0; i < cloud_->size(); ++i) 
      {
        const auto &pt = cloud_->points[i];
        xyz[0] = pt.x; 
        xyz[1] = pt.y; 
        xyz[2] = pt.z; 
        global_mapper_ptr_->occupancy_grid_.WorldToGrid(xyz, ixyz);
        float occupancy_value = global_mapper_ptr_->occupancy_grid_.ReadValue(ixyz);
        bool is_occupied = global_mapper_ptr_->occupancy_grid_.IsOccupied(occupancy_value); 
        bool is_dynamic = global_mapper_ptr_->temporal_grid_.IsDynamic(ixyz, is_occupied);

        // Check bounds 
        if (!global_mapper_ptr_->occupancy_grid_.IsInMap(ixyz)){
          continue; 
        }

        // Populate point clouds
        if (is_occupied)
        {
          global_mapper_ptr_->occupancy_grid_.GridToWorld(ixyz, xyz);
          if (is_dynamic)
          {
            if (xyz[2] > params_.z_ground) // only publish points above the ground
            {
              local_dynamic_cloud.push_back(pcl::PointXYZ(pt.x, pt.y, pt.z)); 
            }
          } else { 
            if (xyz[2] > params_.z_ground) // only publish points above the ground
            {
              local_static_cloud.push_back(pcl::PointXYZ(pt.x, pt.y, pt.z)); 
            }
          }
        }
      }

      #pragma omp critical
      {
        dynamic_cloud += local_dynamic_cloud; 
        static_cloud += local_static_cloud;
      }
    }

    // Publish clouds 
    pcl::toROSMsg(dynamic_cloud, *dynamic_pointcloud);
    dynamic_pointcloud->header.frame_id = params_.global_frame;
    dynamic_pointcloud->header.stamp = pc_stamp_;

    pcl::toROSMsg(static_cloud, *static_pointcloud);
    static_pointcloud->header.frame_id = params_.global_frame;
    static_pointcloud->header.stamp = pc_stamp_;
}                                    

  // might be good to add more debug warnings, e.g dense and sparse paths are emtpy
  void GlobalMapperRos::Publish()
  {
    double prev_time = this->now().seconds();
    voxel_grid::VoxelGrid<float> occupancy_grid;
    voxel_grid::VoxelGrid<int> distance_grid;
    voxel_grid::VoxelGrid<int> cost_grid;
    voxel_grid::VoxelGrid<std::vector<double>> temporal_grid;

    global_mapper_ptr_->GetVoxelGrids(&occupancy_grid, &distance_grid, &cost_grid, &temporal_grid);

    if (params_.publish_occupancy_grid)
    {
      sensor_msgs::msg::PointCloud2 occ_pointcloud_msg;
      PopulateOccupancyPointCloudMsg(occupancy_grid, &occ_pointcloud_msg);
      occ_grid_pub_->publish(occ_pointcloud_msg);
    }

    if (params_.publish_unknown_grid)
    {
      sensor_msgs::msg::PointCloud2 unknown_pointcloud_msg;
      PopulateUnknownPointCloudMsg(occupancy_grid, &unknown_pointcloud_msg);
      unknown_grid_pub_->publish(unknown_pointcloud_msg);
    }

    if (params_.publish_distance_grid)
    {
      sensor_msgs::msg::PointCloud2 dist_pointcloud_msg;
      PopulateDistancePointCloudMsg(distance_grid, &dist_pointcloud_msg);
      dist_grid_pub_->publish(dist_pointcloud_msg);
    }

    if (params_.publish_cost_grid)
    {
      sensor_msgs::msg::PointCloud2 cost_pointcloud_msg;
      PopulateCostPointCloudMsg(cost_grid, &cost_pointcloud_msg);
      cost_grid_pub_->publish(cost_pointcloud_msg);
    }

    if (params_.publish_path)
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

    if (params_.publish_dynamic_grid)
    {
      sensor_msgs::msg::PointCloud2 dynamic_pointcloud_msg;
      sensor_msgs::msg::PointCloud2 static_pointcloud_msg;
      PopulateDynamicPointCloudMsg(occupancy_grid, temporal_grid, &dynamic_pointcloud_msg, &static_pointcloud_msg);
      dynamic_grid_pub_->publish(dynamic_pointcloud_msg);
      if (params_.publish_static_grid)
        static_grid_pub_->publish(static_pointcloud_msg);
    }

    if (params_.verbose) 
    {
      double duration = 1000 * (this->now().seconds() - prev_time);
      std::cout << "\nMap population duration: " << duration << " ms" << std::endl; 
    }
  }

  // Callback for Odometry (jackal)
  void GlobalMapperRos::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_ptr)
  {
    RCLCPP_INFO(this->get_logger(), "In OdomCallback ########################");
    double xyz[3] = {odom_ptr->pose.pose.position.x, odom_ptr->pose.pose.position.y, odom_ptr->pose.pose.position.z};

    if (!std::isfinite(xyz[0]) || !std::isfinite(xyz[1]) || !std::isfinite(xyz[2]))
    {
      RCLCPP_WARN(this->get_logger(), "Received invalid odometry position. Skipping update.");
      return;
    }

    if (!got_pose_)
    {
      got_pose_ = true;
    }
    global_mapper_ptr_->UpdateOrigin(xyz);
  }

  void GlobalMapperRos::PoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_ptr)
  {

    const std::string target_frame = params_.global_frame;
    geometry_msgs::msg::TransformStamped tf_stamped;
    try
    {
      tf_stamped = tf_buffer_ptr_->lookupTransform(
          target_frame,
          params_.odom_frame,
          rclcpp::Time(0),
          rclcpp::Duration(std::chrono::milliseconds(20)));
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(),
                  "[PointCloudCallback] lookupTransform failed: %s", ex.what());
      return;
    }

    // Build Eigen matrix, guard NaN/Inf, cast to float
    Eigen::Matrix4d mat_d = tf2::transformToEigen(tf_stamped).matrix();
    if (!mat_d.allFinite())
    {
      RCLCPP_WARN(this->get_logger(),
                  "Transform matrix contains NaN/Inf, skipping cloud");
      return;
    }
    Eigen::Matrix4f mat_f = mat_d.cast<float>();

    // Transform pose to global frame 
    Eigen::Vector4f xyz_homo(pose_ptr->pose.position.x, pose_ptr->pose.position.y, pose_ptr->pose.position.z, 1.0f);
    Eigen::Vector4f xyz_global = mat_f * xyz_homo;

    double xyz[3] = {xyz_global[0], xyz_global[1], xyz_global[2]};
    if (!std::isfinite(xyz[0]) || !std::isfinite(xyz[1]) || !std::isfinite(xyz[2]))
    {
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
    double xyz[3] = {goal_ptr->pose.position.x, goal_ptr->pose.position.y, params_.target_altitude};
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

  void GlobalMapperRos::PointCloudCallback(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg)
  {
    double prev_time = this->now().seconds(); 

    pc_stamp_ = cloud_msg->header.stamp; 

    // 1) Convert ROS2 PointCloud2 -> PCL PointCloud<PointXYZI> 
    auto tmp = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::fromROSMsg(*cloud_msg, *tmp);

    if (!got_depth_image_)
    {
      got_depth_image_ = true;
    }

    if (params_.verbose)
      std::cout << "Mapper: received cloud with " << tmp->size() << " points" << std::endl; 


    // 2) Downsample using VoxelGrid 
    pcl::PointCloud<pcl::PointXYZI> cloud_filtered;
    if (params_.downsample)
    {
      pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
      voxel_filter.setInputCloud(tmp);
      voxel_filter.setLeafSize(params_.cloud_ds_size, params_.cloud_ds_size, params_.cloud_ds_size);
      voxel_filter.filter(cloud_filtered);
    }
    else 
    {
      cloud_filtered.points = tmp->points; 
    }

    auto in = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    in->points.reserve(cloud_filtered.size());

    for (const auto &pt : cloud_filtered.points)
    {
      pcl::PointXYZI pti;
      pti.x = pt.x;
      pti.y = pt.y;
      pti.z = pt.z;
      pti.intensity = 1.0f; // mark every point as a “hit”
      in->points.push_back(pti);
    }

    // 3) Look up cloud to map and sensor to map transforms (they are not the same for deskewed point clouds)
    const std::string target_frame = params_.global_frame;
    std::string source_frame = cloud_msg->header.frame_id;
    if (!source_frame.empty() && source_frame[0] == '/')  // For hardware, we need to remove transform name slash
        source_frame.erase(0, 1);

    geometry_msgs::msg::TransformStamped cloud_to_map_tf;
    geometry_msgs::msg::TransformStamped sensor_to_map_tf;

    try
    {
      cloud_to_map_tf = tf_buffer_ptr_->lookupTransform(
          target_frame,
          source_frame,
          rclcpp::Time(0),
          rclcpp::Duration(std::chrono::milliseconds(20)));
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(),
                  "[PointCloudCallback] lookupTransform failed: %s", ex.what());
      return;
    }

    source_frame = params_.sensor_frame;

    try
    {
      sensor_to_map_tf = tf_buffer_ptr_->lookupTransform(
          target_frame,
          source_frame,
          rclcpp::Time(0),
          rclcpp::Duration(std::chrono::milliseconds(20)));
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(),
                  "[PointCloudCallback] lookupTransform failed: %s", ex.what());
      return;
    }

    // 4) Build Eigen matrix, guard NaN/Inf, cast to float
    Eigen::Matrix4d mat_d = tf2::transformToEigen(cloud_to_map_tf).matrix();
    if (!mat_d.allFinite())
    {
      RCLCPP_WARN(this->get_logger(),
                  "Transform matrix contains NaN/Inf, skipping cloud");
      return;
    }
    Eigen::Matrix4f mat_f = mat_d.cast<float>();

    // 5) Manually apply transform (avoid PCL’s FPE)
    auto world_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    world_cloud->points.reserve(in->points.size());
    for (const auto &pt : in->points)
    {
      Eigen::Vector4f v(pt.x, pt.y, pt.z, 1.0f);
      Eigen::Vector4f vt = mat_f * v;
      pcl::PointXYZI wpt;
      wpt.x = vt.x();
      wpt.y = vt.y();
      wpt.z = vt.z();
      wpt.intensity = pt.intensity;
      world_cloud->points.push_back(wpt);
    }
    RCLCPP_DEBUG(this->get_logger(),
                 "World cloud has %zu points", world_cloud->points.size());

    // 6) Fill sensor origin for ray tracing using sensor to map transform
    world_cloud->sensor_origin_ << sensor_to_map_tf.transform.translation.x,
        sensor_to_map_tf.transform.translation.y,
        sensor_to_map_tf.transform.translation.z,
        1.0f;

    // 7) Push into mapper
    global_mapper_ptr_->PushPointCloud(world_cloud, this->now().seconds());

    // 8) Copy cloud pointer 
    pcl::copyPointCloud(*world_cloud, *cloud_);

    if (params_.verbose)
    {
      double duration = 1000 * (this->now().seconds() - prev_time);
      std::cout << "\nPC callback (tfs) duration: " << duration << " ms" << std::endl; 
    }
  }

  void GlobalMapperRos::Run()
  {
    GetParams();
    InitSubscribers();
    InitPublishers();

    // Prepend namespace to transform frames 
    params_.odom_frame = name_drone_ + "/" + params_.odom_frame;
    params_.sensor_frame = name_drone_ + "/" + params_.sensor_frame;

    // ── create & store the ProcessStatus on the heap ──
    // `shared_from_this()` is your node pointer
    this->process_status_ =
        std::make_shared<fla_utils::ProcessStatus>(
            this->shared_from_this(), // your node::SharedPtr
            44,                       // ID
            2.0                       // publish rate (Hz)
        );

    // initial state
    this->process_status_->SetStatus(
        fla_interfaces::msg::ProcessStatus::READY);
    this->process_status_->SetArg(0);

    // start your mapping thread
    global_mapper_ptr_ = std::make_unique<
        global_mapper::GlobalMapper>(params_);

    global_mapper_ptr_->Run(); 
    
    // ── spin loop ──
    rclcpp::Rate spin_rate(100.0);
    while (rclcpp::ok())
    {      
      if (!got_pose_)
      {
        this->process_status_->SetStatus(
            fla_interfaces::msg::ProcessStatus::ALARM);
        this->process_status_->SetArg(ProcessArgs::NO_POSE);
      }
      else if (!got_goal_)
      {
        this->process_status_->SetStatus(
            fla_interfaces::msg::ProcessStatus::ALARM);
        this->process_status_->SetArg(ProcessArgs::NO_GOAL);
      }
      else if (!got_depth_image_)
      {
        this->process_status_->SetStatus(
            fla_interfaces::msg::ProcessStatus::ALARM);
        this->process_status_->SetArg(ProcessArgs::NO_DEPTH_IMAGE);
      }
      else
      {
        this->process_status_->SetStatus(
            fla_interfaces::msg::ProcessStatus::READY);
        this->process_status_->SetArg(ProcessArgs::NOMINAL);
      }

      rclcpp::spin_some(this->shared_from_this());
      spin_rate.sleep();
    }

    
  }

} // namespace global_mapper_ros

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<global_mapper_ros::GlobalMapperRos>();
  RCLCPP_INFO(node->get_logger(), "Global Mapper ROS Loop Started.");
  node->Run();

  rclcpp::shutdown();

  return 0;
}