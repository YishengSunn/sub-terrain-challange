#include <functional>

#include <mission_state_machine/mission_state_machine_node.hpp>


MissionStateMachine::MissionStateMachine()
: Node("mission_state_machine"),
  current_target_index_(0),
  has_pose_(false),
  state_(State::WAIT_FOR_POSE),
  last_published_mission_target_index_(0),
  enable_exploration_(true),
  cave_entrance_target_index_(4) {
  this->declare_parameter("reach_threshold", 0.3);
  this->declare_parameter("enable_exploration", true);
  this->declare_parameter("cave_entrance_target_index", 4);

  this->get_parameter("reach_threshold", reach_threshold_);
  this->get_parameter("enable_exploration", enable_exploration_);

  exploration_mode_pub_ =
    this->create_publisher<std_msgs::msg::Bool>("/exploration_mode", rclcpp::QoS(10).transient_local());
  mission_target_index_pub_ =
    this->create_publisher<std_msgs::msg::UInt32>("/mission/target_index", rclcpp::QoS(10).transient_local());

  pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/current_state_est",
    10,
    std::bind(&MissionStateMachine::poseCallback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(200),
    std::bind(&MissionStateMachine::timerCallback, this));

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

  publishMissionTargetIndex(std::numeric_limits<uint32_t>::max());

  RCLCPP_INFO(this->get_logger(), "Mission state machine started.");
  RCLCPP_INFO(this->get_logger(), "Total targets: %ld", targets_.size());
  RCLCPP_INFO(
    this->get_logger(),
    "Exploration after target index %zu (enable=%s)",
    cave_entrance_target_index_,
    enable_exploration_ ? "true" : "false");
}

void MissionStateMachine::publishMissionTargetIndex(uint32_t idx) {
  if (idx == last_published_mission_target_index_) return;

  last_published_mission_target_index_ = idx;
  std_msgs::msg::UInt32 msg;
  msg.data = idx;
  mission_target_index_pub_->publish(msg);
}

void MissionStateMachine::enterMissionComplete() {
  if (state_ == State::DONE) return;

  state_ = State::DONE;
  publishMissionTargetIndex(std::numeric_limits<uint32_t>::max());
  RCLCPP_INFO(this->get_logger(), "Mission complete.");
  if (timer_) timer_->cancel();
}

void MissionStateMachine::enterExplorationMode() {
  if (state_ == State::EXPLORE) return;

  state_ = State::EXPLORE;
  publishMissionTargetIndex(std::numeric_limits<uint32_t>::max());

  std_msgs::msg::Bool msg;
  msg.data = true;
  exploration_mode_pub_->publish(msg);
  RCLCPP_INFO(
    this->get_logger(),
    "Reached cave entrance → autonomous exploration mode (/exploration_mode=true)");
}

void MissionStateMachine::initTargets() {
  const double x0 = -38.0;
  const double y0 = 10.0;
  const double z0 = 7.0;
  const double x1 = -323.0;
  const double y1 = 5.0;
  const double z1 = 12.1;

  for (int i = 0; i < 5; ++i) {
    const double t = static_cast<double>(i) / 4.0;
    geometry_msgs::msg::Point p;
    p.x = x0 + t * (x1 - x0);
    p.y = y0 + t * (y1 - y0);
    p.z = z0 + t * (z1 - z0);
    targets_.push_back(p);
  }
}

void MissionStateMachine::poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_pose_ = msg->pose.pose;
  has_pose_ = true;
}

double MissionStateMachine::distanceToTarget(const geometry_msgs::msg::Point & target) {
  double dx = current_pose_.position.x - target.x;
  double dy = current_pose_.position.y - target.y;
  double dz = current_pose_.position.z - target.z;

  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void MissionStateMachine::timerCallback() {
  switch (state_) {
    case State::WAIT_FOR_POSE: {
      if (!has_pose_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for current pose...");
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Got pose → NAVIGATE");
      state_ = State::NAVIGATE;
      publishMissionTargetIndex(static_cast<uint32_t>(current_target_index_));
      break;
    }

    case State::NAVIGATE: {
      if (current_target_index_ >= targets_.size()) {
        enterMissionComplete();
        return;
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
          publishMissionTargetIndex(static_cast<uint32_t>(current_target_index_));
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

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MissionStateMachine>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
