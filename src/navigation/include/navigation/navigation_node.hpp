#ifndef NAVIGATION__NAVIGATION_NODE_HPP_
#define NAVIGATION__NAVIGATION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <uav_interfaces/srv/nav_set_goal.hpp>
#include <uav_interfaces/srv/nav_add_waypoint.hpp>
#include <uav_interfaces/srv/nav_start.hpp>
#include <uav_interfaces/srv/nav_stop.hpp>
#include <uav_interfaces/srv/nav_get_status.hpp>
#include <uav_interfaces/srv/nav_clear_waypoints.hpp>
#include <uav_interfaces/srv/move.hpp>
#include <uav_interfaces/msg/uav_state.hpp>
#include <uav_interfaces/msg/uav_pose.hpp>

#include <memory>
#include <vector>
#include <string>
#include <mutex>
#include <thread>
#include <atomic>

#include "navigation/obstacle_detector.hpp"
#include "navigation/local_planner.hpp"

namespace navigation
{

struct Waypoint
{
  double x;
  double y;
  double z;
  double yaw;
  double hold_time;
};

class NavigationNode : public rclcpp::Node
{
public:
  explicit NavigationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~NavigationNode();

private:
  // ========== 服务回调 ==========
  void handleSetGoal(
    const std::shared_ptr<uav_interfaces::srv::NavSetGoal::Request> request,
    std::shared_ptr<uav_interfaces::srv::NavSetGoal::Response> response);

  void handleAddWaypoint(
    const std::shared_ptr<uav_interfaces::srv::NavAddWaypoint::Request> request,
    std::shared_ptr<uav_interfaces::srv::NavAddWaypoint::Response> response);

  void handleStart(
    const std::shared_ptr<uav_interfaces::srv::NavStart::Request> request,
    std::shared_ptr<uav_interfaces::srv::NavStart::Response> response);

  void handleStop(
    const std::shared_ptr<uav_interfaces::srv::NavStop::Request> request,
    std::shared_ptr<uav_interfaces::srv::NavStop::Response> response);

  void handleGetStatus(
    const std::shared_ptr<uav_interfaces::srv::NavGetStatus::Request> request,
    std::shared_ptr<uav_interfaces::srv::NavGetStatus::Response> response);

  void handleClearWaypoints(
    const std::shared_ptr<uav_interfaces::srv::NavClearWaypoints::Request> request,
    std::shared_ptr<uav_interfaces::srv::NavClearWaypoints::Response> response);

  // ========== 订阅回调 ==========
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void poseCallback(const uav_interfaces::msg::UavPose::SharedPtr msg);
  void stateCallback(const uav_interfaces::msg::UavState::SharedPtr msg);

  // ========== 定时器回调 ==========
  void planningTimerCallback();
  void visualizationTimerCallback();

  // ========== 导航逻辑 ==========
  void navigationLoop();
  bool flyToPosition(double x, double y, double z, double yaw);
  void stopNavigation();

  // ========== 可视化 ==========
  void publishObstacleMarkers();

  // ========== 参数 ==========
  void declareParameters();
  void getParameters();

  // ========== 服务 ==========
  rclcpp::Service<uav_interfaces::srv::NavSetGoal>::SharedPtr srv_set_goal_;
  rclcpp::Service<uav_interfaces::srv::NavAddWaypoint>::SharedPtr srv_add_waypoint_;
  rclcpp::Service<uav_interfaces::srv::NavStart>::SharedPtr srv_start_;
  rclcpp::Service<uav_interfaces::srv::NavStop>::SharedPtr srv_stop_;
  rclcpp::Service<uav_interfaces::srv::NavGetStatus>::SharedPtr srv_get_status_;
  rclcpp::Service<uav_interfaces::srv::NavClearWaypoints>::SharedPtr srv_clear_waypoints_;

  // ========== 客户端 ==========
  rclcpp::Client<uav_interfaces::srv::Move>::SharedPtr move_client_;

  // ========== 订阅 ==========
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<uav_interfaces::msg::UavPose>::SharedPtr pose_sub_;
  rclcpp::Subscription<uav_interfaces::msg::UavState>::SharedPtr state_sub_;

  // ========== 发布 ==========
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // ========== 定时器 ==========
  rclcpp::TimerBase::SharedPtr planning_timer_;
  rclcpp::TimerBase::SharedPtr visualization_timer_;

  // ========== 模块 ==========
  std::unique_ptr<ObstacleDetector> obstacle_detector_;
  std::unique_ptr<LocalPlanner> local_planner_;

  // ========== 状态 ==========
  std::mutex mutex_;
  uav_interfaces::msg::UavPose::SharedPtr current_pose_;
  uav_interfaces::msg::UavState::SharedPtr current_state_;
  std::vector<Obstacle> current_obstacles_;

  std::vector<Waypoint> waypoints_;
  size_t current_waypoint_index_ = 0;

  enum class NavState
  {
    IDLE,
    NAVIGATING,
    PAUSED,
    AVOIDING,
    COMPLETED,
    ERROR
  };
  std::atomic<NavState> state_{NavState::IDLE};

  std::thread nav_thread_;
  std::atomic<bool> stop_flag_{false};

  // 目标点（单点导航）
  double goal_x_ = 0.0;
  double goal_y_ = 0.0;
  double goal_z_ = 0.0;
  double goal_yaw_ = 0.0;
  bool has_goal_ = false;

  // 参数
  std::string pointcloud_topic_;
  double planning_rate_;
  double visualization_rate_;
};

}  // namespace navigation

#endif  // NAVIGATION__NAVIGATION_NODE_HPP_
