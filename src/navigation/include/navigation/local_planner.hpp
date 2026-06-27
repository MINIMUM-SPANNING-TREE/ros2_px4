#ifndef NAVIGATION__LOCAL_PLANNER_HPP_
#define NAVIGATION__LOCAL_PLANNER_HPP_

#include <vector>
#include <cmath>
#include <algorithm>
#include "navigation/obstacle_detector.hpp"

namespace navigation
{

struct VelocityCommand
{
  double vx;        // 前向速度 (m/s)
  double vy;        // 右向速度 (m/s)
  double vz;        // 上向速度 (m/s)
  double yaw_rate;  // 偏航角速度 (rad/s)
};

class LocalPlanner
{
public:
  struct Config
  {
    double safety_distance;
    double warning_distance;
    int sector_count;
    double max_speed;
    double max_yaw_rate;
    double goal_weight;
    double safety_weight;
    double smooth_weight;

    Config()
      : safety_distance(2.0), warning_distance(3.0),
        sector_count(36), max_speed(2.0), max_yaw_rate(0.5),
        goal_weight(1.0), safety_weight(2.0), smooth_weight(0.5) {}
  };

  explicit LocalPlanner(const Config & config = Config{});

  VelocityCommand plan(
    double goal_x, double goal_y, double goal_z,
    double current_x, double current_y, double current_z,
    double current_yaw,
    const std::vector<Obstacle> & obstacles);

  bool isPathClear(
    double start_x, double start_y, double start_z,
    double end_x, double end_y, double end_z,
    const std::vector<Obstacle> & obstacles);

  void setConfig(const Config & config);
  Config getConfig() const;

private:
  Config config_;
  std::vector<double> sector_centers_;
  double last_vx_ = 0.0;
  double last_vy_ = 0.0;
  double last_yaw_rate_ = 0.0;

  void initSectors();

  std::vector<double> computeSectorCosts(
    const std::vector<Obstacle> & obstacles,
    double drone_x, double drone_y, double drone_z,
    double drone_yaw);

  VelocityCommand emergencyAvoid(
    const Obstacle & obstacle,
    double current_yaw);

  VelocityCommand computeVelocity(
    double best_angle,
    double dist_to_goal,
    double dz,
    const Obstacle * nearest_obstacle,
    double current_yaw);

  VelocityCommand smoothCommand(const VelocityCommand & cmd);
};

}  // namespace navigation

#endif  // NAVIGATION__LOCAL_PLANNER_HPP_
