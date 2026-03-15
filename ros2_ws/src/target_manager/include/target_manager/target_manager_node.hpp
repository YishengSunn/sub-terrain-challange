#pragma once
#include <cmath>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

class TargetManager : public rclcpp::Node {
public:
  TargetManager();

private:
  void initTargets();
  void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  double distanceToTarget(const geometry_msgs::msg::Point & target);
  void publishCurrentTarget();
  void timerCallback();

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<geometry_msgs::msg::Point> targets_;
  size_t current_target_index_;

  geometry_msgs::msg::Pose current_pose_;
  bool has_pose_;
  bool target_sent_;
  double reach_threshold_;
};
