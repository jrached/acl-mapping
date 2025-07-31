// Copyright 2017 Massachusetts Institute of Technology
#pragma once

#include <thread>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include "fla_interfaces/msg/process_status.hpp"

namespace fla_utils {

class ProcessStatus : public rclcpp::Node {
 public:
  ProcessStatus(const uint8_t id, const double rate)
    : Node("process_status"),
      id_(id),
      publish_rate_(rate),
      arg_(0),
      status_(0) {
    SetArg(0);
    SetStatus(0);
    status_thread_ = std::thread(&ProcessStatus::Heartbeat, this);
    
    status_pub_ = this->create_publisher<fla_interfaces::msg:ProcessStatus>("/globalstatus", 0);
    status_pub_->publish<fla_interfaces::msg:ProcessStatus>("/globalstatus", 0);
  }

  void SetArg(uint8_t arg) {
    std::lock_guard<std::mutex> lg(mutex_);
    arg_ = arg;
  }

  void SetStatus(uint8_t status) {
    std::lock_guard<std::mutex> lg(mutex_);
    status_ = status;
  }

private:
  void Heartbeat() {
    while (rclcpp::ok()) {
      fla_interfaces::msg::ProcessStatus ps;
      ps.id = id_;
      ps.pid = getpid();

      std::unique_lock<std::mutex> ul(mutex_);
      ps.status = status_;
      ps.arg = arg_;
      ul.unlock();
      
      status_pub_.publish(ps);

      publish_rate_.sleep();
    }

    return;
  }

  std::mutex mutex_;
  rclcpp::Rate publish_rate_;
  std::thread status_thread_;
  uint8_t id_;
  uint8_t status_;
  uint8_t arg_;
  // ros::Publisher status_pub_;
  rclcpp::Publisher<fla_interfaces::msg:ProcessStatus>SharedPtr status_pub_;
};
}  // namespace fla_utils
