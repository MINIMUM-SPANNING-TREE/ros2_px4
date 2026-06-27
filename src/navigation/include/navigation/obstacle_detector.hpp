#ifndef NAVIGATION__OBSTACLE_DETECTOR_HPP_
#define NAVIGATION__OBSTACLE_DETECTOR_HPP_

#include <vector>
#include <cmath>
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>

namespace navigation
{

struct Obstacle
{
  double x;           // 障碍物中心 X (ENU, m)
  double y;           // 障碍物中心 Y (ENU, m)
  double z;           // 障碍物中心 Z (ENU, m)
  double distance;    // 到无人机的距离 (m)
  double angle;       // 相对于无人机航向的角度 (rad)
  double radius;      // 障碍物近似半径 (m)
  int point_count;    // 构成障碍物的点数
};

class ObstacleDetector
{
public:
  struct Config
  {
    double min_height;
    double max_height;
    double min_distance;
    double max_distance;
    double cluster_tolerance;
    int min_cluster_size;
    int max_cluster_size;

    Config()
      : min_height(-1.0), max_height(3.0),
        min_distance(0.5), max_distance(10.0),
        cluster_tolerance(0.5), min_cluster_size(5),
        max_cluster_size(1000) {}
  };

  explicit ObstacleDetector(const Config & config = Config{});

  std::vector<Obstacle> detect(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    double drone_x, double drone_y, double drone_z,
    double drone_yaw);

  void setConfig(const Config & config);
  Config getConfig() const;

private:
  Config config_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr filterCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
    double drone_x, double drone_y, double drone_z);

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusterCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);

  std::vector<Obstacle> extractObstacles(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & clusters,
    double drone_x, double drone_y, double drone_z,
    double drone_yaw);
};

}  // namespace navigation

#endif  // NAVIGATION__OBSTACLE_DETECTOR_HPP_
