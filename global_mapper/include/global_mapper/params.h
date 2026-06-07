// Copyright 2017 Massachusetts Institute of Technology
#pragma once

#include <vector>
#include <string>

namespace global_mapper
{
struct Params
{
  std::string global_frame = "map";
  std::vector<double> origin = { 0, 0, 0 };
  std::vector<double> world_dimensions = { 5.0, 5.0, 1.0 };
  double resolution = 1.0;
  double radius_drone = 0.15;
  double z_ground = 0;
  int skip = 0;
  double depth_max = 10;
  double r1 = 0.8;
  double r2 = 8.0;
  double z_min_unknown = 0.2;
  double z_max_unknown = 5.0;
  double target_altitude = 0.0;
  double cloud_ds_size = 0.1;
  std::string odom_frame;
  std::string sensor_frame;

  // occupancy_grid
  double init_value = 0;
  double hit_inc = 0.2;
  double miss_inc = -0.01;
  double occupancy_threshold = 0.6;
  bool publish_occupancy_grid = true;
  bool publish_unknown_grid = false;
  double clear_unknown_distance = 5.0;

  // distance_grid
  int truncation_distance = 6;  // in voxels
  bool publish_distance_grid = false;

  // cost_grid
  int inflation_distance = 4;
  int altitude_weight = 20;
  int inflation_weight = 0;
  int unknown_weight = 20;
  int obstacle_weight = 10000;
  bool publish_cost_grid = false;
  bool publish_path = false;

  // temporal grid 
  float occupied_thresh = 3.0; 
  float unoccupied_thresh = 0.5;
  int neighbor_radius = 1; 
  int static_neighbor_thresh = 1; 
  bool publish_dynamic_grid = true;
  bool publish_static_grid = false;
  
};
}  // namespace global_mapper
