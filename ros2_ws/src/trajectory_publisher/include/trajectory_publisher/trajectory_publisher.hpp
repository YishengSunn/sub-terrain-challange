#ifndef TRAJECTORY_PUBLISHER_HPP
#define TRAJECTORY_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>

#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>

#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <trajectory_publisher/polynomial_trajectory.hpp>

class TrajectoryPublisher : public rclcpp::Node {
public:
    TrajectoryPublisher();

private:
    void publishTrajectory();
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void targetCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

    rclcpp::Publisher<
        trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr publisher_;

    rclcpp::Subscription<
        nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::Subscription<
        geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    bool initialized_;

    double odom_x_;
    double odom_y_;
    double odom_z_;

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
