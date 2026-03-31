#include "target_manager/target_manager_node.hpp"

TargetManager::TargetManager() 
: Node("target_manager"),
  current_target_index_(0),
  has_pose_(false),
  state_(State::WAIT_FOR_POSE),
  last_target_pub_time_(rclcpp::Time(0)),
  target_pub_interval_(10.0),
  first_publish_delay_sec_(5.0),
  enable_exploration_(true),
  cave_entrance_target_index_(0) {

    this->declare_parameter("reach_threshold", 0.3);
    this->declare_parameter("target_pub_interval", 10.0);
    this->declare_parameter("first_publish_delay_sec", 5.0);
    this->declare_parameter("enable_exploration", true);
    this->declare_parameter("cave_entrance_target_index", 0);
    this->get_parameter("reach_threshold", reach_threshold_);
    this->get_parameter("target_pub_interval", target_pub_interval_);
    this->get_parameter("first_publish_delay_sec", first_publish_delay_sec_);
    this->get_parameter("enable_exploration", enable_exploration_);

    target_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/target_position", 10);
    exploration_mode_pub_ = this->create_publisher<std_msgs::msg::Bool>("/exploration_mode", rclcpp::QoS(10).transient_local());

    pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/current_state_est",
        10,
        std::bind(&TargetManager::poseCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&TargetManager::timerCallback, this));

    initTargets();

    int cave_idx = 0;
    this->get_parameter("cave_entrance_target_index", cave_idx);

    if (targets_.empty()) {
        cave_entrance_target_index_ = 0;
        enable_exploration_ = false;
        RCLCPP_WARN(this->get_logger(), "No targets; exploration disabled.");
    }

    else {
        if (cave_idx < 0) {
            cave_idx = 0;
        }
        if (static_cast<size_t>(cave_idx) >= targets_.size()) {
            RCLCPP_WARN(
                this->get_logger(),
                "cave_entrance_target_index %d out of range, using last target index %zu",
                cave_idx, targets_.size() - 1);
            cave_idx = static_cast<int>(targets_.size() - 1);
        }
        cave_entrance_target_index_ = static_cast<size_t>(cave_idx);
    }

    RCLCPP_INFO(this->get_logger(), "Target Manager started.");
    RCLCPP_INFO(this->get_logger(), "Total targets: %ld", targets_.size());
    RCLCPP_INFO(
        this->get_logger(),
        "Exploration after target index %zu (enable=%s)",
        cave_entrance_target_index_,
        enable_exploration_ ? "true" : "false");
}

void TargetManager::enterMissionComplete() {
    if (state_ == State::DONE) {
        return;
    }
    state_ = State::DONE;
    RCLCPP_INFO(this->get_logger(), "Mission complete.");
    if (timer_) {
        timer_->cancel();
    }
}

void TargetManager::enterExplorationMode() {
    if (state_ == State::EXPLORE) {
        return;
    }
    state_ = State::EXPLORE;
    std_msgs::msg::Bool msg;
    msg.data = true;
    exploration_mode_pub_->publish(msg);
    RCLCPP_INFO(
        this->get_logger(),
        "Reached cave entrance → autonomous exploration mode (/exploration_mode=true)");
}

void TargetManager::initTargets() {

    geometry_msgs::msg::Point p;

    p.x = -323.008;
    p.y = 4.9993;
    p.z = 11.9416;

    targets_.push_back(p);
}

void TargetManager::poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {

    current_pose_ = msg->pose.pose;
    has_pose_ = true;
}

double TargetManager::distanceToTarget(const geometry_msgs::msg::Point & target) {
    
    double dx = current_pose_.position.x - target.x;
    double dy = current_pose_.position.y - target.y;
    double dz = current_pose_.position.z - target.z;

    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void TargetManager::publishCurrentTarget() {

    if (current_target_index_ >= targets_.size()) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "All targets completed.");
        return;
    }

    geometry_msgs::msg::PointStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "world";
    msg.point = targets_[current_target_index_];

    target_pub_->publish(msg);

    RCLCPP_INFO(
        this->get_logger(),
        "Publishing target: [%.3f, %.3f, %.3f]",
        msg.point.x,
        msg.point.y,
        msg.point.z);
}

void TargetManager::timerCallback() {

    switch (state_) {
        case State::WAIT_FOR_POSE: {
            if (!has_pose_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                    "Waiting for current pose...");
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Got pose → NAVIGATE");
            state_ = State::NAVIGATE;
            break;
        }

        case State::NAVIGATE: {
            if (current_target_index_ >= targets_.size()) {
                enterMissionComplete();
                return;
            }

            rclcpp::Time now = this->now();

            if (last_target_pub_time_.nanoseconds() == 0) {
                if (now.seconds() > first_publish_delay_sec_) {
                    publishCurrentTarget();
                    last_target_pub_time_ = now;
                }
            }
            
            else {
                if ((now - last_target_pub_time_).seconds() > target_pub_interval_) {
                    publishCurrentTarget();
                    last_target_pub_time_ = now;
                }
            }

            {
                double dist = distanceToTarget(targets_[current_target_index_]);

                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000,
                    "Distance to target: %.2f", dist);

                if (dist < reach_threshold_) {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "Reached target (dist=%.3f)", dist);

                    if (enable_exploration_ &&
                        current_target_index_ == cave_entrance_target_index_) {
                        enterExplorationMode();
                        return;
                    }

                    current_target_index_++;
                    if (current_target_index_ >= targets_.size()) {
                        enterMissionComplete();
                        return;
                    }
                }
            }
            break;
        }

        case State::EXPLORE: {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Autonomous exploration active.");
            break;
        }

        case State::DONE:
            break;
    }
}

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);

    auto node = std::make_shared<TargetManager>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
