#ifndef DGP_UTILS_HPP
#define DGP_UTILS_HPP
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
// #include <dgp/data_utils.hpp>
// #include <dgp/termcolor.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
// #include "dynus/dynus_type.hpp"
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


// struct state
// {

//   // time stamp
//   double t = 0.0;

//   // pos, vel, accel, jerk, yaw, dyaw
//   Eigen::Vector3d pos = Eigen::Vector3d::Zero();
//   Eigen::Vector3d vel = Eigen::Vector3d::Zero();
//   Eigen::Vector3d accel = Eigen::Vector3d::Zero();
//   Eigen::Vector3d jerk = Eigen::Vector3d::Zero();
//   double yaw = 0.0;
//   double dyaw = 0.0;

//   // flag for tracking
//   bool use_tracking_yaw = false;

//   void setTimeStamp(const double data)
//   {
//     t = data;
//   }

//   void setPos(const double x, const double y, const double z)
//   {
//     pos << x, y, z;
//   }
//   void setVel(const double x, const double y, const double z)
//   {
//     vel << x, y, z;
//   }
//   void setAccel(const double x, const double y, const double z)
//   {
//     accel << x, y, z;
//   }

//   void setJerk(const double x, const double y, const double z)
//   {
//     jerk << x, y, z;
//   }

//   void setPos(const Eigen::Vector3d &data)
//   {
//     pos << data.x(), data.y(), data.z();
//   }

//   void setVel(const Eigen::Vector3d &data)
//   {
//     vel << data.x(), data.y(), data.z();
//   }

//   void setAccel(const Eigen::Vector3d &data)
//   {
//     accel << data.x(), data.y(), data.z();
//   }

//   void setJerk(const Eigen::Vector3d &data)
//   {
//     jerk << data.x(), data.y(), data.z();
//   }

//   void setState(const Eigen::Vector3d &pos, const Eigen::Vector3d &vel, const Eigen::Vector3d &accel, const Eigen::Vector3d &jerk)
//   {
//     this->pos = pos;
//     this->vel = vel;
//     this->accel = accel;
//     this->jerk = jerk;
//   }

//   void setYaw(const double data)
//   {
//     yaw = data;
//   }

//   void setDYaw(const double data)
//   {
//     dyaw = data;
//   }

//   void setZero()
//   {
//     pos = Eigen::Vector3d::Zero();
//     vel = Eigen::Vector3d::Zero();
//     accel = Eigen::Vector3d::Zero();
//     jerk = Eigen::Vector3d::Zero();
//     yaw = 0;
//     dyaw = 0;
//   }

//   void printPos()
//   {
//     std::cout << "Pos= " << pos.transpose() << std::endl;
//   }

//   void print()
//   {
//     std::cout << "Time= " << t << std::endl;
//     std::cout << "Pos= " << pos.transpose() << std::endl;
//     std::cout << "Vel= " << vel.transpose() << std::endl;
//     std::cout << "Accel= " << accel.transpose() << std::endl;
//   }

//   void printHorizontal()
//   {
//     std::cout << "Pos, Vel, Accel, Jerk= " << pos.transpose() << " " << vel.transpose() << " " << accel.transpose()
//               << " " << jerk.transpose() << std::endl;
//   }
// };

// void printStateDeque(std::deque<state>& data);

// void printStateVector(std::vector<state>& data);

// void vectorOfVectors2MarkerArray(vec_Vecf<3> traj, visualization_msgs::msg::MarkerArray* m_array, std_msgs::msg::ColorRGBA color,
//                                  int type = visualization_msgs::msg::Marker::ARROW,
//                                  std::vector<double> radii = std::vector<double>());

std_msgs::msg::ColorRGBA getColorJet(double v, double vmin, double vmax);

std_msgs::msg::ColorRGBA color(int id);

#endif