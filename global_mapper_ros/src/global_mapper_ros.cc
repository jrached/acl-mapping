// Copyright 2017 Massachusetts Institute of Technology
#include "global_mapper_ros/global_mapper_ros.h"

using namespace std::chrono_literals;

namespace global_mapper_ros
{
  GlobalMapperRos::GlobalMapperRos()
      : Node("global_mapper_ros"), publish_occupancy_grid_(false), publish_distance_grid_(false), publish_cost_grid_(false), publish_path_(false), publish_dynamic_grid_(false), clear_unknown_distance_(0.0), target_altitude_(0.0), start_time_(this->now().seconds()), cloud_(new pcl::PointCloud<pcl::PointXYZ>)
  {

    // og code did not define buffer for some reason
    tf_buffer_ptr_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ptr_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_ptr_);
    name_drone = this->get_namespace();
    name_drone.erase(std::remove(name_drone.begin(), name_drone.end(), '/'), name_drone.end()); // remove slashes
    // lidar_frame_ = name_drone + "/" + name_drone + "_livox";
    lidar_frame_ = name_drone + "/init_pose";
    drone_frame_id_ = name_drone + "/base_link";

    // Instantiate cloud pointer to empty cloud message 
    const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();

  }

  void GlobalMapperRos::GetParams()
  {

    // --- declare all parameters with sensible defaults:
    this->declare_parameter<std::string>("global_frame", "map");
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

    fla_utils::SafeGetParam(*this, "global_frame", params_.global_frame);
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

    // occupancy_grid
    fla_utils::SafeGetParam(*this, "occupancy_grid.init_value", params_.init_value);
    fla_utils::SafeGetParam(*this, "occupancy_grid.hit_inc", params_.hit_inc);
    fla_utils::SafeGetParam(*this, "occupancy_grid.miss_inc", params_.miss_inc);
    fla_utils::SafeGetParam(*this, "occupancy_grid.occupancy_threshold", params_.occupancy_threshold);
    fla_utils::SafeGetParam(*this, "occupancy_grid.publish_unknown_grid", publish_unknown_grid_);
    fla_utils::SafeGetParam(*this, "occupancy_grid.publish_occupancy_grid", publish_occupancy_grid_);
    fla_utils::SafeGetParam(*this, "occupancy_grid.clear_unknown_distance", clear_unknown_distance_);

    // distance_grid
    fla_utils::SafeGetParam(*this, "distance_grid.truncation_distance", params_.truncation_distance);
    fla_utils::SafeGetParam(*this, "distance_grid.publish_distance_grid", publish_distance_grid_);

    // cost_grid
    fla_utils::SafeGetParam(*this, "cost_grid.publish_cost_grid", publish_cost_grid_);
    fla_utils::SafeGetParam(*this, "cost_grid.inflation_distance", params_.inflation_distance);
    fla_utils::SafeGetParam(*this, "cost_grid.publish_path", publish_path_);
    fla_utils::SafeGetParam(*this, "cost_grid.altitude_weight", params_.altitude_weight);
    fla_utils::SafeGetParam(*this, "cost_grid.inflation_weight", params_.inflation_weight);
    fla_utils::SafeGetParam(*this, "cost_grid.unknown_weight", params_.unknown_weight);
    fla_utils::SafeGetParam(*this, "cost_grid.obstacle_weight", params_.obstacle_weight);
    fla_utils::SafeGetParam(*this, "cost_grid.target_altitude", target_altitude_);

    // temporal_grid 
    fla_utils::SafeGetParam(*this, "temporal_grid.publish_dynamic_grid", publish_dynamic_grid_);

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
    RCLCPP_INFO(this->get_logger(), "  occupancy_grid.init_value: %f", params_.init_value);
    RCLCPP_INFO(this->get_logger(), "  occupancy_grid.hit_inc: %f", params_.hit_inc);
    RCLCPP_INFO(this->get_logger(), "  occupancy_grid.miss_inc: %f", params_.miss_inc);
    RCLCPP_INFO(this->get_logger(), "  occupancy_grid.occupancy_threshold: %f", params_.occupancy_threshold);
    RCLCPP_INFO(this->get_logger(), "  occupancy_grid.publish_unknown_grid: %s", publish_unknown_grid_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  occupancy_grid.publish_occupancy_grid: %s", publish_occupancy_grid_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  occupancy_grid.clear_unknown_distance: %f", clear_unknown_distance_);
    RCLCPP_INFO(this->get_logger(), "  distance_grid.truncation_distance: %d", params_.truncation_distance);
    RCLCPP_INFO(this->get_logger(), "  distance_grid.publish_distance_grid: %s", publish_distance_grid_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  cost_grid.publish_cost_grid: %s", publish_cost_grid_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  cost_grid.inflation_distance: %d", params_.inflation_distance);
    RCLCPP_INFO(this->get_logger(), "  cost_grid.publish_path: %s", publish_path_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  cost_grid.altitude_weight: %d", params_.altitude_weight);
    RCLCPP_INFO(this->get_logger(), "  cost_grid.inflation_weight: %d", params_.inflation_weight);
    RCLCPP_INFO(this->get_logger(), "  cost_grid.unknown_weight: %d", params_.unknown_weight);
    RCLCPP_INFO(this->get_logger(), "  cost_grid.obstacle_weight: %d", params_.obstacle_weight);
    RCLCPP_INFO(this->get_logger(), "  cost_grid.target_altitude: %f", target_altitude_);
    RCLCPP_INFO(this->get_logger(), "  temporal_grid.publish_dynamic_grid: %s", publish_dynamic_grid_ ? "true" : "false");
  }

  void GlobalMapperRos::InitSubscribers()
  {
    pose_sub_ = this->create_subscription<dynus_interfaces::msg::State>("pose_topic", 1, std::bind(&GlobalMapperRos::PoseCallback, this, std::placeholders::_1));
    // odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom_topic", 1, std::bind(&GlobalMapperRos::OdomCallback, this, std::placeholders::_1));
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("goal_topic", 1, std::bind(&GlobalMapperRos::GoalCallback, this, std::placeholders::_1));
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("depth_pointcloud_topic", rclcpp::SensorDataQoS(), std::bind(&GlobalMapperRos::PointCloudCallback, this, std::placeholders::_1));
  }

  void GlobalMapperRos::InitPublishers()
  {

    rclcpp::QoS sensor_qos(rclcpp::KeepLast(1));
    sensor_qos.best_effort().durability_volatile();

    if (publish_occupancy_grid_)
    {
      occ_grid_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("occupancy_grid_topic", sensor_qos);
    }

    if (publish_unknown_grid_)
    {
      unknown_grid_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("unknown_grid_topic", 10);
      frontier_grid_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("frontier_grid_topic", 10);
    }

    if (publish_distance_grid_)
    {
      dist_grid_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("distance_grid_topic", 10);
    }

    if (publish_cost_grid_)
    {
      cost_grid_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cost_grid_topic", 10);
    }

    if (publish_path_)
    {
      path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path_topic", 10);
      sparse_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("sparse_path_topic", 10);
    }

    if (publish_dynamic_grid_)
    {
      dynamic_grid_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("dynamic_grid_topic", sensor_qos);
      static_grid_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("static_grid_topic", sensor_qos);
    }

    // i dont understand why this was commented out
    // planning_grids_pub_ = pnh_.advertise<global_mapper_ros::PlanningGrids>("planning_grids", 1);

    grid_pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&GlobalMapperRos::Publish, this));
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
      // std::cout << "Lidar frame: " << lidar_frame_ << std::endl;
      transform_stamped = tf_buffer_ptr_->lookupTransform(params_.global_frame, lidar_frame_, rclcpp::Time(0), 20ms);
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
      pointcloud_frontier->header.frame_id = "map";
      pointcloud_frontier->header.stamp = tstampLastPclFused_;*/

    pcl::toROSMsg(cloud, *pointcloud);
    pointcloud->header.frame_id = "map"; // use world_frame parameter instead
    pointcloud->header.stamp = rclcpp::Clock().now();
    // pointcloud->header.stamp = tstampLastPclFused_;
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
              cloud.push_back(pcl::PointXYZ(xyz[0], xyz[1], xyz[2])); // replace with emplace_back (slightly more optimized according to chat)
            }
          }
        }
      }
    }

    // RCLCPP_INFO(this->get_logger(), "  [Occupancy] found %zu occupied cells", cloud.size());

    pcl::toROSMsg(cloud, *pointcloud);
    pointcloud->header.frame_id = "map";
    pointcloud->header.stamp = rclcpp::Clock().now();
    // I (Jesus) changed the stamp so that it is the same as the last point cloud used in this map
    // pointcloud->header.stamp = tstampLastPclFused_;
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
      transform_stamped = tf_buffer_ptr_->lookupTransform(params_.global_frame, lidar_frame_, rclcpp::Time(0), 20ms);
      transform(0) = transform_stamped.transform.translation.x;
      transform(1) = transform_stamped.transform.translation.y;
      transform(2) = transform_stamped.transform.translation.z;
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "[world_database_master_ros] OnGetTransform failed with %s", ex.what());

      // chat thinks this could be problematic when other functions are called like worldtogrid
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
    // chat mentioned preallocated point cloud for better performance with large grids
    static double max_dist = params_.truncation_distance * params_.truncation_distance;
    for (int x = 0; x < grid_dimensions[0]; x++)
    {
      for (int y = 0; y < grid_dimensions[1]; y++)
      {
        int ixyz[3] = {x, y, slice_ixyz[2]};
        distance_grid.GridToWorld(ixyz, xyz);
        int cost = distance_grid.ReadValue(xyz); // chat claims this is used incorrctly and should instead be ReadValue(ixyz)
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
    pointcloud->header.frame_id = "map";
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
      transform_stamped = tf_buffer_ptr_->lookupTransform(params_.global_frame, lidar_frame_, rclcpp::Time(0), 20ms);
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
    pointcloud->header.frame_id = "map";
    pointcloud->header.stamp = this->now();
  }

  void GlobalMapperRos::PopulatePathMsg(const std::vector<std::array<double, 3>> &path, nav_msgs::msg::Path *path_msg)
  {
    path_msg->header.stamp = this->now();
    path_msg->header.frame_id = "map";
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

    // /////////////// START NEW /////////////////////////////////////////

    // Remove NaN values from the cloud
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_, *cloud_, indices);

    // Voxel grid filtering to downsample the cloud
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud_);
    // vg.setLeafSize(dynus_map_res_, dynus_map_res_, dynus_map_res_);
    vg.setLeafSize(0.2, 0.2, 0.2);
    vg.filter(*cloud_);

    std::unique_lock<std::mutex> lock(global_mapper_ptr_->output_mutex_);

    // Declare clouds to be populated 
    pcl::PointCloud<pcl::PointXYZ> dynamic_cloud;
    pcl::PointCloud<pcl::PointXYZ> static_cloud;

    // Populate clouds according to temporal segmentation scheme
    double xyz[3] = {0.0};
    int ixyz[3] = {0};
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
      if (is_occupied)
      {
        global_mapper_ptr_->occupancy_grid_.GridToWorld(ixyz, xyz);
        if (is_dynamic)
        {
          if (xyz[2] > params_.z_ground) // only publish points above the ground
          {
            dynamic_cloud.push_back(pcl::PointXYZ(xyz[0], xyz[1], xyz[2])); // replace with emplace_back (slightly more optimized according to chat)
          }
        }
        else
        {
          if (xyz[2] > params_.z_ground) // only publish points above the ground
          {
            static_cloud.push_back(pcl::PointXYZ(xyz[0], xyz[1], xyz[2])); // replace with emplace_back (slightly more optimized according to chat)
          }
        }
      }

    }

    lock.unlock();

    // Publish clouds 
    pcl::toROSMsg(dynamic_cloud, *dynamic_pointcloud);
    dynamic_pointcloud->header.frame_id = "map";
    dynamic_pointcloud->header.stamp = rclcpp::Clock().now();

    pcl::toROSMsg(static_cloud, *static_pointcloud);
    static_pointcloud->header.frame_id = "map";
    static_pointcloud->header.stamp = rclcpp::Clock().now();

    // /////////////// END NEW /////////////////////////////////////////


    // std::unique_lock<std::mutex> lock(global_mapper_ptr_->output_mutex_);

    // int grid_dimensions[3];
    // global_mapper_ptr_->occupancy_grid_.GetGridDimensions(grid_dimensions);

    // double xyz[3] = {0.0};
    // pcl::PointCloud<pcl::PointXYZ> dynamic_cloud;
    // pcl::PointCloud<pcl::PointXYZ> static_cloud;
    // for (int x = 0; x < grid_dimensions[0]; x++)
    // {
    //   for (int y = 0; y < grid_dimensions[1]; y++)
    //   {
    //     for (int z = 0; z < grid_dimensions[2]; z++)
    //     {
    //       int ixyz[3] = {x, y, z};
    //       float occupancy_value = global_mapper_ptr_->occupancy_grid_.ReadValue(ixyz);
    //       bool is_occupied = global_mapper_ptr_->occupancy_grid_.IsOccupied(occupancy_value); 
    //       bool is_dynamic = global_mapper_ptr_->temporal_grid_.IsDynamic(ixyz, is_occupied);
    //       if (is_occupied)
    //       {
    //         global_mapper_ptr_->occupancy_grid_.GridToWorld(ixyz, xyz);
    //         if (is_dynamic)
    //         {
    //           if (xyz[2] > params_.z_ground) // only publish points above the ground
    //           {
    //             // std::cout << "Dynamic voxel (" << xyz[0] << ", " << xyz[1] << ", " << xyz[2] << " with timestamp: " << this->now().seconds() - start_time_ << std::endl;
    //             dynamic_cloud.push_back(pcl::PointXYZ(xyz[0], xyz[1], xyz[2])); // replace with emplace_back (slightly more optimized according to chat)
    //           }
    //         }
    //         else
    //         {
    //           if (xyz[2] > params_.z_ground) // only publish points above the ground
    //           {
    //             static_cloud.push_back(pcl::PointXYZ(xyz[0], xyz[1], xyz[2])); // replace with emplace_back (slightly more optimized according to chat)
    //           }
    //         }
    //       }
    //     }
    //   }
    // }

    // lock.unlock();

    // RCLCPP_INFO(this->get_logger(), "  [Occupancy] found %zu occupied cells", dynamic_cloud.size());
    // pcl::toROSMsg(dynamic_cloud, *dynamic_pointcloud);
    // dynamic_pointcloud->header.frame_id = "map";
    // dynamic_pointcloud->header.stamp = rclcpp::Clock().now();

    // pcl::toROSMsg(static_cloud, *static_pointcloud);
    // static_pointcloud->header.frame_id = "map";
    // static_pointcloud->header.stamp = rclcpp::Clock().now();

    // I (Jesus) changed the stamp so that it is the same as the last point cloud used in this map
    // pointcloud->header.stamp = tstampLastPclFused_;

  }                          

  // might be good to add more debug warnings, e.g dense and sparse paths are emtpy
  void GlobalMapperRos::Publish()
  {
    double prev_time = this->now().seconds();
    // get all maps
    voxel_grid::VoxelGrid<float> occupancy_grid;
    voxel_grid::VoxelGrid<int> distance_grid;
    voxel_grid::VoxelGrid<int> cost_grid;
    voxel_grid::VoxelGrid<std::vector<double>> temporal_grid;

    global_mapper_ptr_->GetVoxelGrids(&occupancy_grid, &distance_grid, &cost_grid, &temporal_grid);

    if (publish_occupancy_grid_)
    {
      sensor_msgs::msg::PointCloud2 occ_pointcloud_msg;
      // alternate msg format as suggested by chat
      // auto occ_pointcloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
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

    if (publish_dynamic_grid_)
    {
      sensor_msgs::msg::PointCloud2 dynamic_pointcloud_msg;
      sensor_msgs::msg::PointCloud2 static_pointcloud_msg;
      PopulateDynamicPointCloudMsg(occupancy_grid, temporal_grid, &dynamic_pointcloud_msg, &static_pointcloud_msg);
      dynamic_grid_pub_->publish(dynamic_pointcloud_msg);
      static_grid_pub_->publish(static_pointcloud_msg);
    }

    double duration = 1000 * (this->now().seconds() - prev_time);
    std::cout << "Mapping + segmentation duration: " << duration << " ms" << std::endl; 
  }

  // Callback for Odometry (jackal)
  void GlobalMapperRos::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_ptr)
  {
    // std::cout << "In odom Callback########################" << std::endl;
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

  void GlobalMapperRos::PoseCallback(const dynus_interfaces::msg::State::SharedPtr pose_ptr)
  {

    const std::string target_frame = params_.global_frame;
    geometry_msgs::msg::TransformStamped tf_stamped;
    try
    {
      tf_stamped = tf_buffer_ptr_->lookupTransform(
          target_frame,
          // drone_frame_id_,
          lidar_frame_,
          rclcpp::Time(0),
          rclcpp::Duration(std::chrono::milliseconds(20)));

      // Eigen::Vector3d pos = tf_stamped.transform.translation; 
      // auto quat = tf_stamped.transform.rotation;

      // std::cout << "transform position: (" << tf_stamped.transform.translation.x << ", " << tf_stamped.transform.translation.y << ", " << tf_stamped.transform.translation.z << ")" << std::endl;
      // std::cout << "transform orientation: (" << tf_stamped.transform.rotation.x << ", " << tf_stamped.transform.rotation.y << ", " << tf_stamped.transform.rotation.z << ", " << tf_stamped.transform.rotation.w << ")" << std::endl; 
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(),
                  "[PointCloudCallback] lookupTransform failed: %s", ex.what());
      return;
    }

    // 4) Build Eigen matrix, guard NaN/Inf, cast to float
    Eigen::Matrix4d mat_d = tf2::transformToEigen(tf_stamped).matrix();
    if (!mat_d.allFinite())
    {
      RCLCPP_WARN(this->get_logger(),
                  "Transform matrix contains NaN/Inf, skipping cloud");
      return;
    }
    Eigen::Matrix4f mat_f = mat_d.cast<float>();

    // Transform pose to global frame 
    Eigen::Vector4f xyz_homo(pose_ptr->pos.x, pose_ptr->pos.y, pose_ptr->pos.z, 1.0f);
    Eigen::Vector4f xyz_global = mat_f * xyz_homo;

    // double xyz[3] = {pose_ptr->pos.x, pose_ptr->pos.y, pose_ptr->pos.z};
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
    double xyz[3] = {goal_ptr->pose.position.x, goal_ptr->pose.position.y, target_altitude_};
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
    // 1) Receipt log
    // RCLCPP_INFO(this->get_logger(), "Mapper:: PointCloud received"); // TODO: Uncomment
    if (!got_depth_image_)
    {
      got_depth_image_ = true;
    }

    // 2) Convert ROS2 PointCloud2 → PCL PointCloud<PointXYZI>
    pcl::PointCloud<pcl::PointXYZ> tmp;
    pcl::fromROSMsg(*cloud_msg, tmp);
    auto in = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    in->points.reserve(tmp.size());

    // std::cout << "In PointCloudCallback, tmp.size()=" << tmp.size() << std::endl; // TODO: Uncomment

    for (const auto &pt : tmp.points)
    {
      pcl::PointXYZI pti;
      pti.x = pt.x;
      pti.y = pt.y;
      pti.z = pt.z;
      pti.intensity = 1.0f; // mark every point as a “hit”
      in->points.push_back(pti);
    }

    // 3) Look up cloud → map transform
    const std::string target_frame = params_.global_frame;
    geometry_msgs::msg::TransformStamped tf_stamped;
    try
    {
      tf_stamped = tf_buffer_ptr_->lookupTransform(
          target_frame,
          cloud_msg->header.frame_id,
          rclcpp::Time(0),
          rclcpp::Duration(std::chrono::milliseconds(20)));

      // Eigen::Vector3d pos = tf_stamped.transform.translation; 
      // auto quat = tf_stamped.transform.rotation;
      // std::cout << "Cloud msg frame id: " << cloud_msg->header.frame_id << std::endl;

      // std::cout << "transform position: (" << tf_stamped.transform.translation.x << ", " << tf_stamped.transform.translation.y << ", " << tf_stamped.transform.translation.z << ")" << std::endl;
      // std::cout << "transform orientation: (" << tf_stamped.transform.rotation.x << ", " << tf_stamped.transform.rotation.y << ", " << tf_stamped.transform.rotation.z << ", " << tf_stamped.transform.rotation.w << ")" << std::endl; 
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(),
                  "[PointCloudCallback] lookupTransform failed: %s", ex.what());
      return;
    }

    // 4) Build Eigen matrix, guard NaN/Inf, cast to float
    Eigen::Matrix4d mat_d = tf2::transformToEigen(tf_stamped).matrix();
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

    // 6) Fill sensor origin for ray tracing
    world_cloud->sensor_origin_ << tf_stamped.transform.translation.x,
        tf_stamped.transform.translation.y,
        tf_stamped.transform.translation.z,
        1.0f;

    // 7) Push into mapper
    global_mapper_ptr_->PushPointCloud(world_cloud, this->now().seconds());

    // 8) Update last-fused timestamp
    tstampLastPclFused_ = cloud_msg->header.stamp;

    // 9) Copy cloud pointer 
    pcl::copyPointCloud(*world_cloud, *cloud_);
  }

  void GlobalMapperRos::Run()
  {
    GetParams();
    InitSubscribers();
    InitPublishers();

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
  // std::cout << "Here 5" << std::endl;

  rclcpp::shutdown();

  // std::cout << "Here 6" << std::endl;
  return 0;
}