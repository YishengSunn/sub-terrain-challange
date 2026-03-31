#include <trajectory_publisher/trajectory_publisher.hpp>

TrajectoryPublisher::TrajectoryPublisher()
: Node("trajectory_publisher"), initialized_(false), traj_running_(false) {

    publisher_ = this->create_publisher<
        trajectory_msgs::msg::MultiDOFJointTrajectory>(
        "/command/trajectory", 10);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/current_state_est",
        10,
        std::bind(&TrajectoryPublisher::odomCallback, this, std::placeholders::_1));

    target_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/target_position",
        10,
        std::bind(&TrajectoryPublisher::targetCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&TrajectoryPublisher::publishTrajectory, this));

    RCLCPP_INFO(this->get_logger(), "Trajectory publisher started");
}

void TrajectoryPublisher::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {

    if (!initialized_) {
        target_x_ = msg->pose.pose.position.x;
        target_y_ = msg->pose.pose.position.y;
        target_z_ = msg->pose.pose.position.z;

        initialized_ = true;

        RCLCPP_INFO(
            this->get_logger(),
            "Initial hover position: %.2f %.2f %.2f",
            target_x_, target_y_, target_z_);
    }
}

void TrajectoryPublisher::targetCallback(
    const geometry_msgs::msg::PointStamped::SharedPtr msg) {

    if (!initialized_) return;

    double goal_x = msg->point.x;
    double goal_y = msg->point.y;
    double goal_z = msg->point.z;

    traj_duration_ = 5.0;

    traj_x_.setTrajectory(target_x_, goal_x, traj_duration_);
    traj_y_.setTrajectory(target_y_, goal_y, traj_duration_);
    traj_z_.setTrajectory(target_z_, goal_z, traj_duration_);

    traj_start_time_ = this->now();
    traj_running_ = true;

    RCLCPP_INFO(
        this->get_logger(),
        "New target received: %.2f %.2f %.2f",
        goal_x, goal_y, goal_z);
}

void TrajectoryPublisher::publishTrajectory() {

    if (!initialized_) return;

    if (traj_running_) {

        double t = (this->now() - traj_start_time_).seconds();

        if (t >= traj_duration_) {
            t = traj_duration_;
            traj_running_ = false;
        }

        target_x_ = traj_x_.getPosition(t);
        target_y_ = traj_y_.getPosition(t);
        target_z_ = traj_z_.getPosition(t);
    }

    trajectory_msgs::msg::MultiDOFJointTrajectory msg;

    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "world";

    trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;

    geometry_msgs::msg::Transform transform;

    transform.translation.x = target_x_;
    transform.translation.y = target_y_;
    transform.translation.z = target_z_;

    transform.rotation.x = 0.0;
    transform.rotation.y = 0.0;
    transform.rotation.z = 1.0;
    transform.rotation.w = 0.0;

    geometry_msgs::msg::Twist vel;
    geometry_msgs::msg::Twist acc;

    point.transforms.push_back(transform);
    point.velocities.push_back(vel);
    point.accelerations.push_back(acc);

    point.time_from_start.sec = 0;
    point.time_from_start.nanosec = 100000000;

    msg.points.push_back(point);

    publisher_->publish(msg);
}

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);

    auto node = std::make_shared<TrajectoryPublisher>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
