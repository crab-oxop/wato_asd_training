#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <vector>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void updateMap();
    void integrateCostmap();
    void publishMap();

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::OccupancyGrid latest_costmap_;
    bool costmap_received_ = false;
    bool should_update_map_ = false;

    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_theta_ = 0.0; //Oreitnation
    double last_update_x_ = 0.0;
    double last_update_y_ = 0.0;
    double costmap_x_ = 0.0;
    double costmap_y_ = 0.0;
    double costmap_theta_ = 0.0;
    const double distance_threshold_ = 1.5;

    const double resolution_ = 0.1;
    const int width_ = 300;
    const int height_ = 300;
    const double origin_x_ = -15.0;
    const double origin_y_ = -15.0;

    std::vector<int8_t> global_map_;

    robot::MapMemoryCore map_memory_;
};


#endif 
