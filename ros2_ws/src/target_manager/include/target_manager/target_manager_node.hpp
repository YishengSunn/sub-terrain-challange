#pragma once

#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int32.hpp>


class TargetManager : public rclcpp::Node {
public:
  TargetManager();

private:
  void initTargets();
  void indexCallback(const std_msgs::msg::UInt32::SharedPtr msg);
  void publishCurrentTarget();
  void timerCallback();

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr mission_target_index_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<geometry_msgs::msg::Point> targets_;
  uint32_t mission_target_index_;
  uint32_t last_seen_mission_target_index_;

  rclcpp::Time last_target_pub_time_;
  double target_pub_interval_;
  double first_publish_delay_sec_;
};
