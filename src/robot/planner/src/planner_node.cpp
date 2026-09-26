#include <chrono>
#include <memory>
#include <cmath>
#include <algorithm>

#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
   map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10,
    std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", 10,
    std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10,
    std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = *msg;
  map_received_ = true;

  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  goal_ = *msg;
  goal_received_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
}

void PlannerNode::timerCallback() {
  if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    return;
  }

  if (goalReached()) {
    RCLCPP_INFO(this->get_logger(), "Goal reached.");
    state_ = State::WAITING_FOR_GOAL;
    goal_received_ = false;
    return;
  }

  planPath();
}

bool PlannerNode::goalReached() {
  double dx = goal_.pose.position.x - robot_x_;
  double dy = goal_.pose.position.y - robot_y_;
  return std::sqrt(dx * dx + dy * dy) < goal_tolerance_;
}

bool PlannerNode::worldToGrid(double wx, double wy, CellIndex &idx) {
  double ox = current_map_.info.origin.position.x;
  double oy = current_map_.info.origin.position.y;
  double res = current_map_.info.resolution;

  int gx = static_cast<int>(std::floor((wx - ox) / res));
  int gy = static_cast<int>(std::floor((wy - oy) / res));

  if (gx < 0 || gx >= static_cast<int>(current_map_.info.width) ||
      gy < 0 || gy >= static_cast<int>(current_map_.info.height)) {
    return false;
  }

  idx = CellIndex(gx, gy);
  return true;
}

void PlannerNode::gridToWorld(const CellIndex &idx, double &wx, double &wy) {
  double ox = current_map_.info.origin.position.x;
  double oy = current_map_.info.origin.position.y;
  double res = current_map_.info.resolution;

  wx = ox + (idx.x + 0.5) * res;
  wy = oy + (idx.y + 0.5) * res;
}

bool PlannerNode::isCellFree(const CellIndex &idx) {
  if (idx.x < 0 || idx.x >= static_cast<int>(current_map_.info.width) ||
      idx.y < 0 || idx.y >= static_cast<int>(current_map_.info.height)) {
    return false;
  }

  int8_t value = current_map_.data[idx.y * current_map_.info.width + idx.x];
  return value < obstacle_threshold_;
}

double PlannerNode::heuristic(const CellIndex &a, const CellIndex &b) {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

void PlannerNode::planPath() {
  if (!goal_received_ || !map_received_ || current_map_.data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan: missing map or goal.");
    return;
  }

  CellIndex start_idx;
  CellIndex goal_idx;

  if (!worldToGrid(robot_x_, robot_y_, start_idx) ||
      !worldToGrid(goal_.pose.position.x, goal_.pose.position.y, goal_idx)) {
    RCLCPP_WARN(this->get_logger(), "Start or goal is outside the map.");
    return;
  }

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_map<CellIndex, bool, CellIndexHash> closed;

  const int dx8[8] = {1, -1,  0,  0,  1,  1, -1, -1};
  const int dy8[8] = {0,  0,  1, -1,  1, -1,  1, -1};

  g_score[start_idx] = 0.0;
  open_set.push(AStarNode(start_idx, heuristic(start_idx, goal_idx)));

  bool found = false;

  while (!open_set.empty()) {
    AStarNode current = open_set.top();
    open_set.pop();

    if (closed[current.index]) {
      continue;
    }
    closed[current.index] = true;

    if (current.index == goal_idx) {
      found = true;
      break;
    }

    for (int i = 0; i < 8; ++i) {
      CellIndex neighbour(current.index.x + dx8[i], current.index.y + dy8[i]);

      bool near_start = (std::abs(neighbour.x - start_idx.x) <= 5 &&
                         std::abs(neighbour.y - start_idx.y) <= 5);

      if ((!isCellFree(neighbour) && !near_start) || closed[neighbour]) {
        continue;
      }

      double step_cost = (dx8[i] != 0 && dy8[i] != 0) ? std::sqrt(2.0) : 1.0;
      double tentative_g = g_score[current.index] + step_cost;

      if (g_score.find(neighbour) == g_score.end() || tentative_g < g_score[neighbour]) {
        g_score[neighbour] = tentative_g;
        came_from[neighbour] = current.index;
        open_set.push(AStarNode(neighbour, tentative_g + heuristic(neighbour, goal_idx)));
      }
    }
  }

  if (!found) {
    RCLCPP_WARN(this->get_logger(), "No path found.");
    return;
  }

  std::vector<CellIndex> cells;
  CellIndex c = goal_idx;
  cells.push_back(c);

  while (c != start_idx) {
    c = came_from[c];
    cells.push_back(c);
  }

  std::reverse(cells.begin(), cells.end());

  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = "sim_world";

  for (const CellIndex &cell : cells) {
    double wx, wy;
    gridToWorld(cell, wx, wy);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;

    path.poses.push_back(pose);
  }

  path_pub_->publish(path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
