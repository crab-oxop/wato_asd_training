#include "map_memory_node.hpp"
#include <chrono>
#include <memory>
#include <cmath>

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  global_map_.assign(width_ * height_, 0);

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap",
    10,
    std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));


  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered",
    10,
    std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));


  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000),
    std::bind(&MapMemoryNode::updateMap, this));

  
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;

  costmap_x_ = robot_x_;
  costmap_y_ = robot_y_;
  costmap_theta_ = robot_theta_;

  if (!costmap_received_) {
    should_update_map_ = true;
  }
  costmap_received_ = true;
}
void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;

  double qx = msg->pose.pose.orientation.x;
  double qy = msg->pose.pose.orientation.y;
  double qz = msg->pose.pose.orientation.z;
  double qw = msg->pose.pose.orientation.w;

  robot_theta_ = std::atan2(2.0 * (qw * qz + qx * qy),
                            1.0 - 2.0 * (qy * qy + qz * qz));

  double dx = robot_x_ - last_update_x_;
  double dy = robot_y_ - last_update_y_;

  if (std::sqrt(dx * dx + dy * dy) >= distance_threshold_) {
    last_update_x_ = robot_x_;
    last_update_y_ = robot_y_;
    should_update_map_ = true;
  }
}
void MapMemoryNode::updateMap() {
  if (should_update_map_ && costmap_received_) {
    integrateCostmap();
    should_update_map_ = false;
  }

  publishMap();
}

void MapMemoryNode::integrateCostmap() {
  double cm_resolution = latest_costmap_.info.resolution;
  int cm_width = latest_costmap_.info.width;
  int cm_height = latest_costmap_.info.height;
  double cm_origin_x = latest_costmap_.info.origin.position.x;
  double cm_origin_y = latest_costmap_.info.origin.position.y;

  double cos_theta = std::cos(costmap_theta_);
  double sin_theta = std::sin(costmap_theta_);

  for (int y = 0; y < cm_height; ++y) {
    for (int x = 0; x < cm_width; ++x) {

      int8_t value = latest_costmap_.data[y * cm_width + x];
      if (value <= 0) {
        continue;
      }

      double local_x = cm_origin_x + (x + 0.5) * cm_resolution;
      double local_y = cm_origin_y + (y + 0.5) * cm_resolution;

      double world_x = costmap_x_ + local_x * cos_theta - local_y * sin_theta;
      double world_y = costmap_y_ + local_x * sin_theta + local_y * cos_theta;

      int gx = static_cast<int>(std::floor((world_x - origin_x_) / resolution_));
      int gy = static_cast<int>(std::floor((world_y - origin_y_) / resolution_));

      if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_) {
        continue;
      }

      if (value > global_map_[gy * width_ + gx]) {
        global_map_[gy * width_ + gx] = value;
      }
    }
  }
}
void MapMemoryNode::publishMap() {
  nav_msgs::msg::OccupancyGrid map;

  map.header.stamp = this->get_clock()->now();
  map.header.frame_id = "sim_world";

  map.info.resolution = resolution_;
  map.info.width = width_;
  map.info.height = height_;

  map.info.origin.position.x = origin_x_;
  map.info.origin.position.y = origin_y_;
  map.info.origin.orientation.w = 1.0;

  map.data = global_map_;

  map_pub_->publish(map);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
