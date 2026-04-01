#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int32.hpp>


class MissionStateMachine : public rclcpp::Node {
public:
  MissionStateMachine();

private:
  void initTargets();
  void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  double distanceToTarget(const geometry_msgs::msg::Point & target);
  void publishMissionTargetIndex(uint32_t idx);
  void timerCallback();
  void enterMissionComplete();
  void enterExplorationMode();

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr exploration_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr mission_target_index_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<geometry_msgs::msg::Point> targets_;
  size_t current_target_index_;

  geometry_msgs::msg::Pose current_pose_;
  bool has_pose_;

  enum class State {
    WAIT_FOR_POSE,
    NAVIGATE,
    EXPLORE,
    DONE
  };

  State state_;
  uint32_t last_published_mission_target_index_;

  double reach_threshold_;
  bool enable_exploration_;
  size_t cave_entrance_target_index_;
};
