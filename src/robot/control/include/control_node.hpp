#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include <optional>
#include <cmath>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "control_core.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void controlLoop();

    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint();
    geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::PoseStamped &target);
    double computeDistance(const geometry_msgs::msg::Point &a,
                           const geometry_msgs::msg::Point &b);
    double extractYaw(const geometry_msgs::msg::Quaternion &q);

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Path current_path_;
    nav_msgs::msg::Odometry::SharedPtr robot_odom_;

    const double lookahead_distance_ = 1;
    const double goal_tolerance_ = 0.1;
    const double lin_kp_ = 0.5;
    const double max_lin_vel_ = 0.5;
    const double ang_kp_ = 2.0;

    robot::ControlCore control_;
};

#endif
