#include <chrono>
#include <memory>
#include <cmath>
#include <limits>
#include <algorithm>

#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10,
    std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10,
    std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&ControlNode::controlLoop, this));
}
void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  current_path_ = *msg;
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_odom_ = msg;
}

void ControlNode::controlLoop() {
  if (current_path_.poses.empty() || !robot_odom_) {
    return;
  }

  double dist_to_goal = computeDistance(
    robot_odom_->pose.pose.position,
    current_path_.poses.back().pose.position);

  if (dist_to_goal < goal_tolerance_) {
    geometry_msgs::msg::Twist stop;
    cmd_vel_pub_->publish(stop);
    return;
  }

  std::optional<geometry_msgs::msg::PoseStamped> target = findLookaheadPoint();

  if (!target) {
    geometry_msgs::msg::Twist stop;
    cmd_vel_pub_->publish(stop);
    return;
  }

  cmd_vel_pub_->publish(computeVelocity(*target));
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
  if (current_path_.poses.empty()) {
    return std::nullopt;
  }

  size_t closest_idx = 0;
  double closest_dist = std::numeric_limits<double>::max();

  for (size_t i = 0; i < current_path_.poses.size(); ++i) {
    double d = computeDistance(robot_odom_->pose.pose.position,
                               current_path_.poses[i].pose.position);
    if (d < closest_dist) {
      closest_dist = d;
      closest_idx = i;
    }
  }

  for (size_t i = closest_idx; i < current_path_.poses.size(); ++i) {
    double d = computeDistance(robot_odom_->pose.pose.position,
                               current_path_.poses[i].pose.position);
    if (d >= lookahead_distance_) {
      return current_path_.poses[i];
    }
  }

  return current_path_.poses.back();
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(
    const geometry_msgs::msg::PoseStamped &target) {

  geometry_msgs::msg::Twist cmd;

  double dx = target.pose.position.x - robot_odom_->pose.pose.position.x;
  double dy = target.pose.position.y - robot_odom_->pose.pose.position.y;

  double target_angle = std::atan2(dy, dx);
  double robot_yaw = extractYaw(robot_odom_->pose.pose.orientation);

  double alpha = target_angle - robot_yaw;
  while (alpha > M_PI)  { alpha -= 2.0 * M_PI; }
  while (alpha < -M_PI) { alpha += 2.0 * M_PI; }

  double dist_to_goal = computeDistance(
    robot_odom_->pose.pose.position,
    current_path_.poses.back().pose.position);

  cmd.linear.x = std::min(lin_kp_ * dist_to_goal, max_lin_vel_);
  cmd.linear.x *= std::max(0.3, 1.0 - std::fabs(alpha));
  cmd.angular.z = ang_kp_ * alpha;

  return cmd;

}

double ControlNode::computeDistance(const geometry_msgs::msg::Point &a,
                                    const geometry_msgs::msg::Point &b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
