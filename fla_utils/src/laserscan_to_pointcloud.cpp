// laserscan_to_pointcloud.cpp

#include <cmath>
#include <string>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <laser_geometry/laser_geometry.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <fla_interfaces/msg/process_status.hpp>

class LaserScanToPointCloud : public rclcpp::Node
{

public:
  LaserScanToPointCloud();
  
private:
  laser_geometry::LaserProjection laser_projection_;
  int channel_option_;

  sensor_msgs::msg::LaserScan scan_;
  sensor_msgs::msg::PointCloud2 cloud_;
  rclcpp::Time last_scan_stamp_;

  // Parameters
  bool use_negative_info_;
  int fla_process_id_;

  // ROS interfaces
  // ros::NodeHandle nh_;
  // ros::Subscriber sub_scan_;
  // ros::Publisher  pub_cloud2_;
  // ros::Publisher  pub_status_;
  // ros::Timer      heartbeat_timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud2_;
  rclcpp::Publisher<fla_interfaces::msg::ProcessStatus>::SharedPtr pub_status_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  void scan_handler(const sensor_msgs::msg::LaserScan::SharedPtr scan);
  void heartbeat_callback() const;
  std::size_t set_null_ranges_max(const sensor_msgs::msg::LaserScan& scan_in, sensor_msgs::msg::LaserScan& scan_out);
};

LaserScanToPointCloud::LaserScanToPointCloud() : rclcpp::Node("lasercan_to_pointcloud")
{
  // nh_ = ros::NodeHandle("~");
  // ROS_INFO("[LaserscanToPointcloud] Started.");
  RCLCPP_INFO(this->get_logger(), "[LaserscanToPointcloud] Started.");

  // get required parameters
  bool params_received = true;
  // params_received &= nh_.getParam("use_negative_info", use_negative_info_);
  // params_received &= nh_.getParam("fla_process_id", fla_process_id_);
  
  this->declare_parameter("use_negative_info_", false);
  this->declare_parameter("fla_process_id_", 0);  

  params_received &= this->get_parameter("use_negative_info_", use_negative_info_);
  params_received &= this->get_parameter("fla_process_id_", fla_process_id_);

  if (!params_received)
  {
    // ROS_ERROR("[LaserscanToPointcloud] Missing required parameters. Exiting.");
    // ros::shutdown();

    RCLCPP_ERROR(this->get_logger(), "[LaserscanToPointcloud] Missing required parameters. Exiting.");
    rclcpp::shutdown();
    return;
  }

  channel_option_ = laser_geometry::channel_option::Default;
  last_scan_stamp_ = rclcpp::Time(0);

  // start interfaces
  
  // Publishers and Subscribers
  pub_cloud2_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud2_out", 1);
  pub_status_ = this->create_publisher<fla_interfaces::msg::ProcessStatus>("/globalstatus", 1);
  sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan_in", 1, std::bind(&LaserScanToPointCloud::scan_handler, this, std::placeholders::_1));
  heartbeat_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&LaserScanToPointCloud::heartbeat_callback, this));

  return;
}

void LaserScanToPointCloud::scan_handler(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  last_scan_stamp_ = scan->header.stamp;

  if (use_negative_info_)
  {
    std::size_t num_rays_maxed = set_null_ranges_max(*scan, scan_);
    laser_projection_.projectLaser(scan_, cloud_, -1.0, channel_option_);
  }
  else
  {
    laser_projection_.projectLaser(*scan, cloud_, -1.0, channel_option_);
  }
  
  pub_cloud2_->publish(cloud_);
  return;
}

void LaserScanToPointCloud::heartbeat_callback() const
{
  fla_interfaces::msg::ProcessStatus status;

  status.id = fla_process_id_;
  status.pid = getpid();

  rclcpp::Time t_now = this->now();

  if ( last_scan_stamp_ == rclcpp::Time(0) )
  {
    status.status = fla_interfaces::msg::ProcessStatus::INIT;
    status.arg = 1;
  }
  else if ( t_now - last_scan_stamp_ > rclcpp::Duration::from_seconds(1.0) )
  {
    status.status = fla_interfaces::msg::ProcessStatus::ALARM;
    status.arg = 2;
  }
  else
  {
    status.status = fla_interfaces::msg::ProcessStatus::READY;
    status.arg = 0;
  }

  pub_status_->publish(status);

  return;
}

std::size_t LaserScanToPointCloud::set_null_ranges_max(const sensor_msgs::msg::LaserScan& scan_in, sensor_msgs::msg::LaserScan& scan_out)
{
  std::size_t num_rays_maxed = 0;

  // Copy over relevant data
  scan_out.header           = scan_in.header;
  scan_out.angle_min        = scan_in.angle_min;
  scan_out.angle_max        = scan_in.angle_max;
  scan_out.angle_increment  = scan_in.angle_increment;
  scan_out.time_increment   = scan_in.time_increment;
  scan_out.scan_time        = scan_in.scan_time;
  scan_out.range_min        = scan_in.range_min;
  scan_out.range_max        = scan_in.range_max;
  scan_out.intensities      = scan_in.intensities;

  scan_out.ranges.clear();
  scan_out.ranges.reserve(scan_in.ranges.size());

  // We need the maximum range value that is still LESS than max range
  float max_range = nextafterf(scan_in.range_max, 0.0);

  for (std::size_t i=0; i < scan_in.ranges.size(); ++i)
  {
    if (scan_out.ranges[i] < scan_in.range_min)
    {
      scan_out.ranges.push_back( max_range );
      ++num_rays_maxed;
    }
    else
    {
      scan_out.ranges.push_back( scan_in.ranges[i] );
    }
  }

  return num_rays_maxed;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LaserScanToPointCloud>();
  LaserScanToPointCloud converter;
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}