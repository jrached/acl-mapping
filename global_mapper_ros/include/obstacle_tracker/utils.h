#ifndef DGP_UTILS_HPP
#define DGP_UTILS_HPP
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <deque>

#define RED 1
#define RED_TRANS 2
#define RED_TRANS_TRANS 3
#define GREEN 4
#define BLUE 5
#define BLUE_TRANS 6
#define BLUE_TRANS_TRANS 7
#define BLUE_LIGHT 8
#define YELLOW 9
#define ORANGE_TRANS 10
#define BLACK_TRANS 11
#define ORANGE 12
#define GREEN_TRANS_TRANS 13

#define STATE 0
#define INPUT 1

#define WHOLE_TRAJ 0
#define RESCUE_PATH 1

#define OCCUPIED_SPACE 1
#define UNKOWN_AND_OCCUPIED_SPACE 2

std_msgs::msg::ColorRGBA getColorJet(double v, double vmin, double vmax);

std_msgs::msg::ColorRGBA color(int id);

#endif