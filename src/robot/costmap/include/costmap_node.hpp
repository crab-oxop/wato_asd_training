#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();

  private:
    void initializeCostmap();
    void convertToGrid(double range, double angle, int &x_grid, int &y_grid);
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void markObstacle(int x_grid, int y_grid);
    void inflateObstacles();
    void publishCostmap();

    const double resolution_ = 0.1;
    const int width_ = 300;
    const int height_ = 300;
    const double origin_x_ = -15.0;
    const double origin_y_ = -15.0;

    std::vector<int8_t> costmap_data_;
    std_msgs::msg::Header scan_header_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    robot::CostmapCore costmap_;
};

#endif 