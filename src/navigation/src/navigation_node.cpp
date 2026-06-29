#include "navigation/navigation_node.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace navigation
{

NavigationNode::NavigationNode(const rclcpp::NodeOptions & options)
: Node("navigation_node", options)
{
  // 声明和获取参数
  declareParameters();
  getParameters();

  // 初始化模块
  ObstacleDetector::Config det_config;
  obstacle_detector_ = std::make_unique<ObstacleDetector>(det_config);

  LocalPlanner::Config plan_config;
  plan_config.safety_distance = this->get_parameter("safety_distance").as_double();
  plan_config.warning_distance = this->get_parameter("warning_distance").as_double();
  plan_config.max_speed = this->get_parameter("max_speed").as_double();
  plan_config.max_yaw_rate = this->get_parameter("max_yaw_rate").as_double();
  local_planner_ = std::make_unique<LocalPlanner>(plan_config);

  // 服务
  // 设置单个目标点
  srv_set_goal_ = this->create_service<uav_interfaces::srv::NavSetGoal>(
    "nav/set_goal", std::bind(&NavigationNode::handleSetGoal, this, _1, _2));
  // 向航点列表末尾添加一个航点
  srv_add_waypoint_ = this->create_service<uav_interfaces::srv::NavAddWaypoint>(
    "nav/add_waypoint", std::bind(&NavigationNode::handleAddWaypoint, this, _1, _2));
  // 开始执行导航
  srv_start_ = this->create_service<uav_interfaces::srv::NavStart>(
    "nav/start", std::bind(&NavigationNode::handleStart, this, _1, _2));
  // 停止导航
  srv_stop_ = this->create_service<uav_interfaces::srv::NavStop>(
    "nav/stop", std::bind(&NavigationNode::handleStop, this, _1, _2));
  // 获取导航状态
  srv_get_status_ = this->create_service<uav_interfaces::srv::NavGetStatus>(
    "nav/get_status", std::bind(&NavigationNode::handleGetStatus, this, _1, _2));
  // 清空航点列表
  srv_clear_waypoints_ = this->create_service<uav_interfaces::srv::NavClearWaypoints>(
    "nav/clear_waypoints", std::bind(&NavigationNode::handleClearWaypoints, this, _1, _2));

  // 客户端
  move_client_ = this->create_client<uav_interfaces::srv::Move>("uav/move");

  // 订阅
  auto sensor_qos = rclcpp::QoS(5).best_effort();

  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    pointcloud_topic_, sensor_qos,
    std::bind(&NavigationNode::pointcloudCallback, this, _1));

  pose_sub_ = this->create_subscription<uav_interfaces::msg::UavPose>(
    "/uav/telemetry/pose", 10,
    std::bind(&NavigationNode::poseCallback, this, _1));

  state_sub_ = this->create_subscription<uav_interfaces::msg::UavState>(
    "/uav/telemetry/state", 10,
    std::bind(&NavigationNode::stateCallback, this, _1));

  // 发布
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/navigation/markers", 10);
  velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    "/mavros/setpoint_velocity/cmd_vel", 10);

  // 定时器
  planning_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / planning_rate_),
    std::bind(&NavigationNode::planningTimerCallback, this));

  visualization_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / visualization_rate_),
    std::bind(&NavigationNode::visualizationTimerCallback, this));

  RCLCPP_INFO(this->get_logger(), "========================================");
  RCLCPP_INFO(this->get_logger(), "  UAV Navigation Node Started (C++)");
  RCLCPP_INFO(this->get_logger(), "========================================");
  RCLCPP_INFO(this->get_logger(), "  点云话题: %s", pointcloud_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  安全距离: %.1f m", plan_config.safety_distance);
  RCLCPP_INFO(this->get_logger(), "  最大速度: %.1f m/s", plan_config.max_speed);
  RCLCPP_INFO(this->get_logger(), "  规划频率: %.1f Hz", planning_rate_);
  RCLCPP_INFO(this->get_logger(), "========================================");
}

NavigationNode::~NavigationNode()
{
  stopNavigation();
}

void NavigationNode::declareParameters()
{
  this->declare_parameter("pointcloud_topic", "/lslidar_point_cloud");
  this->declare_parameter("safety_distance", 2.0);
  this->declare_parameter("warning_distance", 3.0);
  this->declare_parameter("max_speed", 2.0);
  this->declare_parameter("max_yaw_rate", 0.5);
  this->declare_parameter("planning_rate", 10.0);
  this->declare_parameter("visualization_rate", 5.0);
  this->declare_parameter("default_timeout", 60.0);
}

void NavigationNode::getParameters()
{
  pointcloud_topic_ = this->get_parameter("pointcloud_topic").as_string();
  planning_rate_ = this->get_parameter("planning_rate").as_double();
  visualization_rate_ = this->get_parameter("visualization_rate").as_double();
}

// ========== 服务回调 ==========

void NavigationNode::handleSetGoal(
  const std::shared_ptr<uav_interfaces::srv::NavSetGoal::Request> request,
  std::shared_ptr<uav_interfaces::srv::NavSetGoal::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);

  goal_x_ = request->x;
  goal_y_ = request->y;
  goal_z_ = request->z;
  goal_yaw_ = request->yaw;
  has_goal_ = true;

  // 清空航点，使用单点目标
  waypoints_.clear();
  Waypoint wp;
  wp.x = request->x;
  wp.y = request->y;
  wp.z = request->z;
  wp.yaw = request->yaw;
  wp.hold_time = 0.0;
  waypoints_.push_back(wp);
  current_waypoint_index_ = 0;

  response->success = true;
  response->message = "目标点已设置: (" +
    std::to_string(request->x) + ", " +
    std::to_string(request->y) + ", " +
    std::to_string(request->z) + ")";

  RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
}

void NavigationNode::handleAddWaypoint(
  const std::shared_ptr<uav_interfaces::srv::NavAddWaypoint::Request> request,
  std::shared_ptr<uav_interfaces::srv::NavAddWaypoint::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);

  Waypoint wp;
  wp.x = request->x;
  wp.y = request->y;
  wp.z = request->z;
  wp.yaw = request->yaw;
  wp.hold_time = request->hold_time;

  waypoints_.push_back(wp);
  int idx = static_cast<int>(waypoints_.size()) - 1;

  response->success = true;
  response->waypoint_index = idx;
  response->message = "航点已添加 [" + std::to_string(idx) + "]: (" +
    std::to_string(request->x) + ", " +
    std::to_string(request->y) + ", " +
    std::to_string(request->z) + ")";

  RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
}

void NavigationNode::handleStart(
  const std::shared_ptr<uav_interfaces::srv::NavStart::Request> request,
  std::shared_ptr<uav_interfaces::srv::NavStart::Response> response)
{
  if (waypoints_.empty()) {
    response->success = false;
    response->message = "没有航点，无法开始导航";
    response->total_waypoints = 0;
    RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  if (state_ == NavState::NAVIGATING) {
    response->success = false;
    response->message = "导航已在进行中";
    response->total_waypoints = static_cast<int>(waypoints_.size());
    RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  if (!move_client_->wait_for_service(5s)) {
    response->success = false;
    response->message = "uav/move 服务不可用";
    response->total_waypoints = 0;
    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    return;
  }

  current_waypoint_index_ = 0;
  stop_flag_ = false;
  avoidance_counter_ = 0;
  state_ = NavState::NAVIGATING;

  // 启动导航线程
  if (nav_thread_.joinable()) {
    nav_thread_.join();
  }
  nav_thread_ = std::thread(&NavigationNode::navigationLoop, this);

  response->success = true;
  response->total_waypoints = static_cast<int>(waypoints_.size());
  response->message = "导航已开始，共 " + std::to_string(waypoints_.size()) + " 个航点";

  RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
}

void NavigationNode::handleStop(
  const std::shared_ptr<uav_interfaces::srv::NavStop::Request> request,
  std::shared_ptr<uav_interfaces::srv::NavStop::Response> response)
{
  if (request->pause) {
    if (state_ == NavState::NAVIGATING) {
      state_ = NavState::PAUSED;
      response->success = true;
      response->message = "导航已暂停";
      RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    } else {
      response->success = false;
      response->message = "当前不在导航状态，无法暂停";
    }
  } else {
    stopNavigation();
    response->success = true;
    response->message = "导航已停止";
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
  }
}

void NavigationNode::handleGetStatus(
  const std::shared_ptr<uav_interfaces::srv::NavGetStatus::Request> /*request*/,
  std::shared_ptr<uav_interfaces::srv::NavGetStatus::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // 状态字符串
  switch (state_.load()) {
    case NavState::IDLE:
      response->state = "idle";
      break;
    case NavState::NAVIGATING:
      response->state = "navigating";
      break;
    case NavState::PAUSED:
      response->state = "paused";
      break;
    case NavState::AVOIDING:
      response->state = "avoiding";
      break;
    case NavState::COMPLETED:
      response->state = "completed";
      break;
    case NavState::ERROR:
      response->state = "error";
      break;
  }

  response->current_waypoint = static_cast<int>(current_waypoint_index_);
  response->total_waypoints = static_cast<int>(waypoints_.size());
  response->obstacle_count = static_cast<int>(current_obstacles_.size());
  response->success = true;

  // 当前位置
  if (current_pose_) {
    response->pos_x = current_pose_->x;
    response->pos_y = current_pose_->y;
    response->pos_z = current_pose_->z;
  } else {
    response->pos_x = 0.0;
    response->pos_y = 0.0;
    response->pos_z = 0.0;
  }

  // 到目标距离
  if (!waypoints_.empty() && current_waypoint_index_ < waypoints_.size()) {
    const auto & wp = waypoints_[current_waypoint_index_];
    double dx = wp.x - response->pos_x;
    double dy = wp.y - response->pos_y;
    double dz = wp.z - response->pos_z;
    response->distance_to_goal = std::sqrt(dx * dx + dy * dy + dz * dz);
  } else {
    response->distance_to_goal = 0.0;
  }
}

void NavigationNode::handleClearWaypoints(
  const std::shared_ptr<uav_interfaces::srv::NavClearWaypoints::Request> /*request*/,
  std::shared_ptr<uav_interfaces::srv::NavClearWaypoints::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (state_ == NavState::NAVIGATING) {
    stopNavigation();
  }

  waypoints_.clear();
  current_waypoint_index_ = 0;
  has_goal_ = false;

  response->success = true;
  response->message = "航点已清空";
  RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
}

// ========== 订阅回调 ==========

void NavigationNode::pointcloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // 转换为 PCL 点云
  auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(
    new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);

  // 获取无人机位置
  double drone_x = 0.0, drone_y = 0.0, drone_z = 0.0, drone_yaw = 0.0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (current_pose_) {
      drone_x = current_pose_->x;
      drone_y = current_pose_->y;
      drone_z = current_pose_->z;
      drone_yaw = current_pose_->yaw;
    }
  }

  // 检测障碍物
  auto obstacles = obstacle_detector_->detect(cloud, drone_x, drone_y, drone_z, drone_yaw);

  {
    std::lock_guard<std::mutex> lock(mutex_);
    current_obstacles_ = obstacles;
  }
}

void NavigationNode::poseCallback(
  const uav_interfaces::msg::UavPose::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  current_pose_ = msg;
}

void NavigationNode::stateCallback(
  const uav_interfaces::msg::UavState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  current_state_ = msg;
}

// ========== 定时器回调 ==========

void NavigationNode::planningTimerCallback()
{
  // 避障状态下持续发布速度指令
  if (state_ == NavState::AVOIDING) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!current_pose_ || current_obstacles_.empty()) {
      return;
    }

    if (current_waypoint_index_ >= waypoints_.size()) {
      return;
    }

    const auto & wp = waypoints_[current_waypoint_index_];
    double current_x = current_pose_->x;
    double current_y = current_pose_->y;
    double current_z = current_pose_->z;

    // 高度保护：低于1.5m时强制上升
    if (current_z < 1.5) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "高度过低 (%.1fm)，强制上升", current_z);
      publishVelocity(0.0, 0.0, 0.5, 0.0);  // 强制上升
      return;
    }

    // 执行避障规划
    auto cmd = local_planner_->plan(
      wp.x, wp.y, wp.z,
      current_x, current_y, current_z,
      current_pose_->yaw,
      current_obstacles_);

    // 发布避障速度指令
    publishVelocity(cmd.vx, cmd.vy, cmd.vz, cmd.yaw_rate);

    // 检查路径是否恢复畅通
    if (local_planner_->isPathClear(
        current_x, current_y, current_z,
        wp.x, wp.y, wp.z,
        current_obstacles_))
    {
      avoidance_counter_++;
      if (avoidance_counter_ >= 10) {  // 连续1秒路径畅通
        RCLCPP_INFO(this->get_logger(), "路径恢复畅通，继续导航");
        state_ = NavState::NAVIGATING;
        avoidance_counter_ = 0;
        stopVelocity();
      }
    } else {
      avoidance_counter_ = 0;
    }

    return;
  }

  if (state_ != NavState::NAVIGATING) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  if (!current_pose_ || current_obstacles_.empty()) {
    return;
  }

  if (current_waypoint_index_ >= waypoints_.size()) {
    return;
  }

  const auto & wp = waypoints_[current_waypoint_index_];
  double current_x = current_pose_->x;
  double current_y = current_pose_->y;
  double current_z = current_pose_->z;

  // 检查路径是否畅通
  if (!local_planner_->isPathClear(
      current_x, current_y, current_z,
      wp.x, wp.y, wp.z,
      current_obstacles_))
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "检测到障碍物，启动避障");

    state_ = NavState::AVOIDING;
    avoidance_counter_ = 0;
  }
}

void NavigationNode::visualizationTimerCallback()
{
  publishObstacleMarkers();
}

// ========== 导航逻辑 ==========

void NavigationNode::navigationLoop()
{
  RCLCPP_INFO(this->get_logger(), "导航线程启动");

  while (!stop_flag_ && current_waypoint_index_ < waypoints_.size()) {
    // 暂停检查
    if (state_ == NavState::PAUSED) {
      std::this_thread::sleep_for(100ms);
      continue;
    }

    // 避障检查 - 等待避障完成或超时
    if (state_ == NavState::AVOIDING) {
      avoidance_counter_++;
      if (avoidance_counter_ >= AVOIDANCE_TIMEOUT) {
        RCLCPP_WARN(this->get_logger(), "避障超时，跳过当前航点");
        state_ = NavState::NAVIGATING;
        avoidance_counter_ = 0;
        stopVelocity();
        current_waypoint_index_++;
      }
      std::this_thread::sleep_for(100ms);
      continue;
    }

    // 获取当前航点
    Waypoint wp;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      wp = waypoints_[current_waypoint_index_];
    }

    RCLCPP_INFO(this->get_logger(),
      "导航到航点 [%zu/%zu]: (%.1f, %.1f, %.1f)",
      current_waypoint_index_ + 1, waypoints_.size(),
      wp.x, wp.y, wp.z);

    // 飞向航点
    bool success = flyToPosition(wp.x, wp.y, wp.z, wp.yaw);

    if (success) {
      RCLCPP_INFO(this->get_logger(),
        "到达航点 [%zu]", current_waypoint_index_ + 1);

      // 悬停等待
      if (wp.hold_time > 0) {
        RCLCPP_INFO(this->get_logger(), "悬停 %.1f 秒", wp.hold_time);
        std::this_thread::sleep_for(
          std::chrono::duration<double>(wp.hold_time));
      }

      current_waypoint_index_++;
    } else {
      RCLCPP_WARN(this->get_logger(),
        "航点 [%zu] 导航失败", current_waypoint_index_ + 1);
      current_waypoint_index_++;
    }
  }

  // 导航完成
  if (current_waypoint_index_ >= waypoints_.size()) {
    state_ = NavState::COMPLETED;
    RCLCPP_INFO(this->get_logger(), "所有航点导航完成");
  } else {
    state_ = NavState::IDLE;
  }

  RCLCPP_INFO(this->get_logger(), "导航线程结束");
}

bool NavigationNode::flyToPosition(double x, double y, double z, double yaw)
{
  auto request = std::make_shared<uav_interfaces::srv::Move::Request>();
  request->x = x;
  request->y = y;
  request->z = z;
  request->yaw = yaw;

  auto future = move_client_->async_send_request(request);

  // 等待结果
  auto status = future.wait_for(std::chrono::seconds(60));
  if (status == std::future_status::ready) {
    auto result = future.get();
    return result->success;
  }

  RCLCPP_ERROR(this->get_logger(), "uav/move 服务调用超时");
  return false;
}

void NavigationNode::publishVelocity(double vx, double vy, double vz, double yaw_rate)
{
  auto msg = geometry_msgs::msg::TwistStamped();
  msg.header.stamp = this->now();
  msg.header.frame_id = "";  // MAVROS 期望空帧或 local_origin
  msg.twist.linear.x = vx;
  msg.twist.linear.y = vy;
  msg.twist.linear.z = vz;
  msg.twist.angular.z = yaw_rate;
  velocity_pub_->publish(msg);
}

void NavigationNode::stopVelocity()
{
  publishVelocity(0.0, 0.0, 0.0, 0.0);
}

void NavigationNode::stopNavigation()
{
  stop_flag_ = true;
  state_ = NavState::IDLE;
  stopVelocity();

  if (nav_thread_.joinable()) {
    nav_thread_.join();
  }
}

// ========== 可视化 ==========

void NavigationNode::publishObstacleMarkers()
{
  auto marker_array = visualization_msgs::msg::MarkerArray();

  // 清除旧标记
  visualization_msgs::msg::Marker clear_marker;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);

  // 添加新标记
  std::lock_guard<std::mutex> lock(mutex_);

  for (size_t i = 0; i < current_obstacles_.size(); ++i) {
    const auto & obs = current_obstacles_[i];

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "obstacles";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = obs.x;
    marker.pose.position.y = obs.y;
    marker.pose.position.z = obs.z;

    marker.scale.x = obs.radius * 2;
    marker.scale.y = obs.radius * 2;
    marker.scale.z = 1.0;

    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.5f;

    marker.lifetime = rclcpp::Duration::from_seconds(1.0);

    marker_array.markers.push_back(marker);
  }

  marker_pub_->publish(marker_array);
}

}  // namespace navigation

// ========== 主函数 ==========
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<navigation::NavigationNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
