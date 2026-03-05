#ifndef TRAJECTORY_PUBLISHER_HPP
#define TRAJECTORY_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>

#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>

#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <trajectory_publisher/polynomial_trajectory.hpp>

#include <thread>

class TrajectoryPublisher : public rclcpp::Node
{
public:
    TrajectoryPublisher();

private:
    void publishTrajectory();
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void keyboardLoop();

    rclcpp::Publisher<
        trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr publisher_;

    rclcpp::Subscription<
        nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::thread keyboard_thread_;

    bool initialized_;

    double target_x_;
    double target_y_;
    double target_z_;

    PolynomialTrajectory traj_x_;
    PolynomialTrajectory traj_y_;
    PolynomialTrajectory traj_z_;

    rclcpp::Time traj_start_time_;
    bool traj_running_;
    double traj_duration_;
};

#endif