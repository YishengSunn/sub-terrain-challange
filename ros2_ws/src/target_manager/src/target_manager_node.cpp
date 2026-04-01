#include <functional>

#include <target_manager/target_manager_node.hpp>


TargetManager::TargetManager()
: Node("target_manager"),
  mission_target_index_(std::numeric_limits<uint32_t>::max()),
  last_seen_mission_target_index_(std::numeric_limits<uint32_t>::max()),
  last_target_pub_time_(rclcpp::Time(0)),
  target_pub_interval_(10.0),
  first_publish_delay_sec_(5.0) {
  this->declare_parameter("target_pub_interval", 10.0);
  this->declare_parameter("first_publish_delay_sec", 5.0);
  
  this->get_parameter("target_pub_interval", target_pub_interval_);
  this->get_parameter("first_publish_delay_sec", first_publish_delay_sec_);

  target_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/target_position", 10);

  mission_target_index_sub_ = this->create_subscription<std_msgs::msg::UInt32>(
    "/mission/target_index",
    rclcpp::QoS(10).transient_local(),
    std::bind(&TargetManager::indexCallback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(200),
    std::bind(&TargetManager::timerCallback, this));

  initTargets();

  RCLCPP_INFO(this->get_logger(), "Target manager started (publishes /target_position from /mission/target_index).");
  RCLCPP_INFO(this->get_logger(), "Total targets: %ld", targets_.size());
}

void TargetManager::indexCallback(const std_msgs::msg::UInt32::SharedPtr msg) {
  if (msg->data != last_seen_mission_target_index_) {
    last_target_pub_time_ = rclcpp::Time(0);
    last_seen_mission_target_index_ = msg->data;
  }
  mission_target_index_ = msg->data;
}

void TargetManager::initTargets() {
  // Keep in sync with mission_state_machine::initTargets().
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

void TargetManager::publishCurrentTarget() {
  if (mission_target_index_ >= targets_.size()) return;

  geometry_msgs::msg::PointStamped msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "world";
  msg.point = targets_[mission_target_index_];

  target_pub_->publish(msg);

  RCLCPP_INFO(
    this->get_logger(),
    "Publishing target: [%.3f, %.3f, %.3f]",
    msg.point.x,
    msg.point.y,
    msg.point.z);
}

void TargetManager::timerCallback() {
  if (mission_target_index_ == std::numeric_limits<uint32_t>::max() ||
      mission_target_index_ >= targets_.size()) return;

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
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TargetManager>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
