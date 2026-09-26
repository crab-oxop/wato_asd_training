#include <chrono>
#include <memory>
#include <cmath>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 
    10, 
    std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}
void CostmapNode::initializeCostmap() {
  costmap_data_.assign(width_ * height_, 0);
}
void CostmapNode::markObstacle(int x_grid, int y_grid) {
  if (x_grid < 0 || x_grid >= width_ || y_grid < 0 || y_grid >= height_) {
    return;
  }
  costmap_data_[y_grid * width_ + x_grid] = 100;
}
void CostmapNode::convertToGrid(double range, double angle, int &x_grid, int &y_grid) {
  double x = range * std::cos(angle);
  double y = range * std::sin(angle);

  x_grid = static_cast<int>(std::floor((x - origin_x_) / resolution_));
  y_grid = static_cast<int>(std::floor((y - origin_y_) / resolution_));
}
void CostmapNode::publishCostmap() {
  nav_msgs::msg::OccupancyGrid grid;

  grid.header = scan_header_;

  grid.info.resolution = resolution_;
  grid.info.width = width_;
  grid.info.height = height_;

  grid.info.origin.position.x = origin_x_;
  grid.info.origin.position.y = origin_y_;
  grid.info.origin.orientation.w = 1.0;

  grid.data = costmap_data_;

  costmap_pub_->publish(grid);
}
void CostmapNode::inflateObstacles() {
  const double inflation_radius = 1.5;
  const int max_cost = 100;
  const int cell_radius = static_cast<int>(inflation_radius / resolution_);

  std::vector<int8_t> source = costmap_data_;

  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      if (source[y * width_ + x] != max_cost) {
        continue;
      }

      for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
        for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
          int nx = x + dx;
          int ny = y + dy;

          if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) {
            continue;
          }

          double distance = std::sqrt(dx * dx + dy * dy) * resolution_;
          if (distance > inflation_radius) {
            continue;
          }

          int8_t cost = static_cast<int8_t>(max_cost * (1.0 - distance / inflation_radius));
          if (cost > costmap_data_[ny * width_ + nx]) {
            costmap_data_[ny * width_ + nx] = cost;
          }
        }
      }
    }
  }
}

void CostmapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  initializeCostmap();
  scan_header_ = scan->header;

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double angle = scan->angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];

    if (range < scan->range_max && range > scan->range_min) {
      int x_grid, y_grid;
      convertToGrid(range, angle, x_grid, y_grid);
      markObstacle(x_grid, y_grid);
    }
  }
  inflateObstacles();
  publishCostmap();
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}