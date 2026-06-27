#include "navigation/obstacle_detector.hpp"
#include <algorithm>
#include <numeric>
#include <pcl/common/centroid.h>

namespace navigation
{

ObstacleDetector::ObstacleDetector(const Config & config)
: config_(config)
{
}

void ObstacleDetector::setConfig(const Config & config)
{
  config_ = config;
}

ObstacleDetector::Config ObstacleDetector::getConfig() const
{
  return config_;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ObstacleDetector::filterCloud(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  double drone_x, double drone_y, double drone_z)
{
  auto filtered = pcl::PointCloud<pcl::PointXYZ>::Ptr(
    new pcl::PointCloud<pcl::PointXYZ>);

  for (const auto & point : cloud->points) {
    // 相对高度
    double relative_z = point.z - drone_z;

    // 水平距离
    double dx = point.x - drone_x;
    double dy = point.y - drone_y;
    double dist = std::sqrt(dx * dx + dy * dy);

    // 高度过滤
    if (relative_z < config_.min_height || relative_z > config_.max_height) {
      continue;
    }

    // 距离过滤
    if (dist < config_.min_distance || dist > config_.max_distance) {
      continue;
    }

    filtered->points.push_back(point);
  }

  filtered->width = filtered->points.size();
  filtered->height = 1;
  filtered->is_dense = true;

  return filtered;
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> ObstacleDetector::clusterCloud(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;

  if (cloud->empty()) {
    return clusters;
  }

  // KD-Tree 搜索
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
    new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);

  // 欧氏聚类
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(config_.cluster_tolerance);
  ec.setMinClusterSize(config_.min_cluster_size);
  ec.setMaxClusterSize(config_.max_cluster_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  // 提取聚类
  for (const auto & indices : cluster_indices) {
    auto cluster = pcl::PointCloud<pcl::PointXYZ>::Ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto & idx : indices.indices) {
      cluster->points.push_back(cloud->points[idx]);
    }
    cluster->width = cluster->points.size();
    cluster->height = 1;
    cluster->is_dense = true;
    clusters.push_back(cluster);
  }

  return clusters;
}

std::vector<Obstacle> ObstacleDetector::extractObstacles(
  const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & clusters,
  double drone_x, double drone_y, double drone_z,
  double drone_yaw)
{
  std::vector<Obstacle> obstacles;

  for (const auto & cluster : clusters) {
    if (cluster->empty()) {
      continue;
    }

    // 计算聚类中心
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cluster, centroid);

    double cx = centroid[0];
    double cy = centroid[1];
    double cz = centroid[2];

    // 到无人机的距离
    double dx = cx - drone_x;
    double dy = cy - drone_y;
    double dist = std::sqrt(dx * dx + dy * dy);

    // 相对于无人机航向的角度
    double angle = std::atan2(dy, dx) - drone_yaw;
    // 归一化到 [-pi, pi]
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;

    // 计算障碍物半径
    double max_radius = 0.0;
    for (const auto & point : cluster->points) {
      double r = std::sqrt(
        (point.x - cx) * (point.x - cx) +
        (point.y - cy) * (point.y - cy));
      if (r > max_radius) {
        max_radius = r;
      }
    }

    Obstacle obs;
    obs.x = cx;
    obs.y = cy;
    obs.z = cz;
    obs.distance = dist;
    obs.angle = angle;
    obs.radius = max_radius;
    obs.point_count = static_cast<int>(cluster->size());

    obstacles.push_back(obs);
  }

  // 按距离排序
  std::sort(obstacles.begin(), obstacles.end(),
    [](const Obstacle & a, const Obstacle & b) {
      return a.distance < b.distance;
    });

  return obstacles;
}

std::vector<Obstacle> ObstacleDetector:: detect(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
  double drone_x, double drone_y, double drone_z,
  double drone_yaw)
{
  if (cloud->empty()) {
    return {};
  }

  // 步骤1: 滤波
  auto filtered = filterCloud(cloud, drone_x, drone_y, drone_z);
  if (filtered->empty()) {
    return {};
  }

  // 步骤2: 聚类
  auto clusters = clusterCloud(filtered);

  // 步骤3: 提取障碍物
  return extractObstacles(clusters, drone_x, drone_y, drone_z, drone_yaw);
}

}  // namespace navigation
