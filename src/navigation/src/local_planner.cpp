#include "navigation/local_planner.hpp"
#include <algorithm>
#include <limits>
#include <cmath>

namespace navigation
{

LocalPlanner::LocalPlanner(const Config & config)
: config_(config)
{
  initSectors();
}

void LocalPlanner::setConfig(const Config & config)
{
  config_ = config;
  initSectors();
}

LocalPlanner::Config LocalPlanner::getConfig() const
{
  return config_;
}

void LocalPlanner::initSectors()
{
  sector_centers_.resize(config_.sector_count);
  for (int i = 0; i < config_.sector_count; ++i) {
    sector_centers_[i] = -M_PI + (2.0 * M_PI * i) / config_.sector_count;
  }
}

std::vector<double> LocalPlanner::computeSectorCosts(
  const std::vector<Obstacle> & obstacles,
  double drone_x, double drone_y, double drone_z,
  double drone_yaw)
{
  std::vector<double> costs(config_.sector_count, 0.0);

  for (const auto & obs : obstacles) {
    // 距离代价
    double dist_cost = 0.0;
    if (obs.distance < config_.safety_distance) {
      dist_cost = 1.0;
    } else if (obs.distance < config_.warning_distance) {
      dist_cost = (config_.warning_distance - obs.distance) /
                  (config_.warning_distance - config_.safety_distance);
    }

    // 障碍物角宽度
    double half_angular_width = 0.0;
    if (obs.distance > 0) {
      half_angular_width = std::atan2(
        obs.radius + config_.safety_distance * 0.5,
        obs.distance);
    } else {
      half_angular_width = M_PI;
    }

    // 分配到扇区
    for (int i = 0; i < config_.sector_count; ++i) {
      double angle_diff = std::abs(sector_centers_[i] - obs.angle);
      if (angle_diff > M_PI) {
        angle_diff = 2 * M_PI - angle_diff;
      }

      if (angle_diff <= half_angular_width) {
        costs[i] = std::max(costs[i], dist_cost);
      }
    }
  }

  return costs;
}

VelocityCommand LocalPlanner::emergencyAvoid(
  const Obstacle & obstacle,
  double current_yaw)
{
  VelocityCommand cmd;
  cmd.vx = -0.3;
  cmd.vy = 0.0;
  cmd.vz = 0.0;

  if (obstacle.angle > 0) {
    cmd.yaw_rate = -config_.max_yaw_rate;
  } else {
    cmd.yaw_rate = config_.max_yaw_rate;
  }

  return cmd;
}

VelocityCommand LocalPlanner::computeVelocity(
  double best_angle,
  double dist_to_goal,
  double dz,
  const Obstacle * nearest_obstacle,
  double current_yaw)
{
  VelocityCommand cmd;

  // 速度大小
  double speed = config_.max_speed;
  if (dist_to_goal < 1.0) {
    speed = config_.max_speed * dist_to_goal;
  }

  // 附近有障碍物时减速
  if (nearest_obstacle && nearest_obstacle->distance < config_.warning_distance) {
    double speed_factor = nearest_obstacle->distance / config_.warning_distance;
    speed *= std::max(speed_factor, 0.3);
  }

  cmd.vx = speed * std::cos(best_angle);
  cmd.vy = speed * std::sin(best_angle);
  cmd.vz = std::clamp(dz * 0.5, -0.5, 0.5);
  cmd.yaw_rate = std::clamp(best_angle * 0.5, -config_.max_yaw_rate, config_.max_yaw_rate);

  return cmd;
}

VelocityCommand LocalPlanner::smoothCommand(const VelocityCommand & cmd)
{
  double alpha = config_.smooth_weight;

  VelocityCommand smoothed;
  smoothed.vx = cmd.vx * (1 - alpha) + last_vx_ * alpha;
  smoothed.vy = cmd.vy * (1 - alpha) + last_vy_ * alpha;
  smoothed.vz = cmd.vz;
  smoothed.yaw_rate = cmd.yaw_rate * (1 - alpha) + last_yaw_rate_ * alpha;

  last_vx_ = smoothed.vx;
  last_vy_ = smoothed.vy;
  last_yaw_rate_ = smoothed.yaw_rate;

  return smoothed;
}

VelocityCommand LocalPlanner::plan(
  double goal_x, double goal_y, double goal_z,
  double current_x, double current_y, double current_z,
  double current_yaw,
  const std::vector<Obstacle> & obstacles)
{
  // 计算到目标的距离和角度
  double dx = goal_x - current_x;
  double dy = goal_y - current_y;
  double dz = goal_z - current_z;
  double dist_to_goal = std::sqrt(dx * dx + dy * dy);
  double goal_angle = std::atan2(dy, dx);
  double goal_relative_angle = goal_angle - current_yaw;
  while (goal_relative_angle > M_PI) goal_relative_angle -= 2 * M_PI;
  while (goal_relative_angle < -M_PI) goal_relative_angle += 2 * M_PI;

  // 计算扇区代价
  auto sector_costs = computeSectorCosts(
    obstacles, current_x, current_y, current_z, current_yaw);

  // 计算总代价
  std::vector<double> total_costs(config_.sector_count);
  for (int i = 0; i < config_.sector_count; ++i) {
    double angle_diff = std::abs(sector_centers_[i] - goal_relative_angle);
    if (angle_diff > M_PI) angle_diff = 2 * M_PI - angle_diff;
    double goal_cost = angle_diff / M_PI;

    total_costs[i] = config_.goal_weight * goal_cost +
                     config_.safety_weight * sector_costs[i];
  }

  // 选择最优扇区
  int best_idx = 0;
  double min_cost = std::numeric_limits<double>::max();
  for (int i = 0; i < config_.sector_count; ++i) {
    if (total_costs[i] < min_cost) {
      min_cost = total_costs[i];
      best_idx = i;
    }
  }
  double best_angle = sector_centers_[best_idx];

  // 检查紧急避障
  double half_angle = M_PI / 6.0;
  const Obstacle * nearest = nullptr;
  double min_dist = std::numeric_limits<double>::max();

  for (const auto & obs : obstacles) {
    double angle_diff = std::abs(obs.angle - best_angle);
    if (angle_diff > M_PI) angle_diff = 2 * M_PI - angle_diff;

    if (angle_diff <= half_angle && obs.distance < min_dist) {
      min_dist = obs.distance;
      nearest = &obs;
    }
  }

  if (nearest && nearest->distance < config_.safety_distance) {
    return smoothCommand(emergencyAvoid(*nearest, current_yaw));
  }

  // 计算速度指令
  VelocityCommand cmd = computeVelocity(best_angle, dist_to_goal, dz, nearest, current_yaw);
  return smoothCommand(cmd);
}

bool LocalPlanner::isPathClear(
  double start_x, double start_y, double start_z,
  double end_x, double end_y, double end_z,
  const std::vector<Obstacle> & obstacles)
{
  double dx = end_x - start_x;
  double dy = end_y - start_y;
  double path_length = std::sqrt(dx * dx + dy * dy);

  if (path_length < 0.1) {
    return true;
  }

  double ux = dx / path_length;
  double uy = dy / path_length;

  for (const auto & obs : obstacles) {
    double ox = obs.x - start_x;
    double oy = obs.y - start_y;

    double proj = ox * ux + oy * uy;

    if (proj >= 0 && proj <= path_length) {
      double perp_dist = std::abs(ox * uy - oy * ux);
      if (perp_dist < obs.radius + config_.safety_distance) {
        return false;
      }
    }
  }

  return true;
}

}  // namespace navigation
