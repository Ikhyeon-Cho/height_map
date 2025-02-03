/*
 * HeightMapping.cpp
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_ros/core/HeightMapper.h"

namespace height_mapping_ros {
HeightMapper::HeightMapper(const Config &cfg)
    : cfg_{cfg}, heightFilter_{cfg.min_height, cfg.max_height} {

  paramValidityCheck();

  initHeightMap();

  initHeightEstimator();
}

void HeightMapper::paramValidityCheck() {

  if (cfg_.grid_resolution <= 0) {
    throw std::invalid_argument(
        "[HeightMapping::HeightMapping]: Grid resolution must be positive");
  }
  if (cfg_.map_length_x <= 0 || cfg_.map_length_y <= 0) {
    throw std::invalid_argument(
        "[HeightMapping::HeightMapping]: Map dimensions must be positive");
  }
}

void HeightMapper::initHeightMap() {
  map_.setFrameId(cfg_.frame_id);
  map_.setGeometry(grid_map::Length(cfg_.map_length_x, cfg_.map_length_y),
                   cfg_.grid_resolution);
}

void HeightMapper::initHeightEstimator() {

  // Set height estimator
  // - Kalman Filter
  // - Moving Average
  // - StatMean (by default)

  if (cfg_.estimator_type == "KalmanFilter") {
    heightEstimator_ = std::make_unique<height_mapping::KalmanEstimator>();

    std::cout << "\033[1;33m[HeightMapping::HeightMapping]: Height estimator "
                 "type --> KalmanFilter \033[0m\n";

  } else if (cfg_.estimator_type == "MovingAverage") {
    heightEstimator_ =
        std::make_unique<height_mapping::MovingAverageEstimator>();

    std::cout << "\033[1;33m[HeightMapping::HeightMapping]: Height estimator "
                 "type --> MovingAverage \033[0m\n";

  } else if (cfg_.estimator_type == "StatMean") {
    heightEstimator_ = std::make_unique<height_mapping::StatMeanEstimator>();

    std::cout << "\033[1;33m[HeightMapping::HeightMapping]: Height estimator "
                 "type --> StatisticalMeanEstimator "
                 "\033[0m\n";

  } else {
    heightEstimator_ = std::make_unique<height_mapping::StatMeanEstimator>();

    std::cout << "\033[1;33m[HeightMapping::HeightMapping] Invalid height "
                 "estimator type. Set Default: StatMeanEstimator \033[0m\n";
  }
}

template <typename PointT>
void HeightMapper::fastHeightFilter(
    const typename pcl::PointCloud<PointT>::Ptr &cloud,
    typename pcl::PointCloud<PointT>::Ptr &filtered_cloud) {

  heightFilter_.filter<PointT>(cloud, filtered_cloud);
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr HeightMapper::griddedFilterWithMaxHeight(
    const typename pcl::PointCloud<PointT>::Ptr &cloud, float gridSize) {

  if (cloud->empty()) {
    return cloud;
  }

  // grid cell: first, second -> (x_index, y_index), point
  std::unordered_map<std::pair<int, int>, PointT, pair_hash> grid_map;
  for (const auto &point : *cloud) {
    int x_index = std::floor(point.x / gridSize);
    int y_index = std::floor(point.y / gridSize);

    auto grid_key = std::make_pair(x_index, y_index);
    auto [iter, inserted] = grid_map.try_emplace(grid_key, point);

    if (!inserted && point.z > iter->second.z) {
      iter->second = point;
    }
  }

  auto cloud_downsampled = boost::make_shared<pcl::PointCloud<PointT>>();
  cloud_downsampled->reserve(grid_map.size());
  for (const auto &grid_cell : grid_map) {
    cloud_downsampled->points.emplace_back(grid_cell.second);
  }

  cloud_downsampled->header = cloud->header;
  return cloud_downsampled;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr
HeightMapper::griddedFilterWithMaxHeightAlt(
    const typename pcl::PointCloud<PointT>::Ptr &cloud, float gridSize) {

  if (cloud->empty()) {
    return cloud;
  }

  std::unordered_map<std::pair<int, int>, PointT, pair_hash> gridCells;
  grid_map::Position measuredPosition;
  grid_map::Index measuredIndex;

  for (const auto &point : *cloud) {
    measuredPosition << point.x, point.y;

    if (!map_.getIndex(measuredPosition, measuredIndex)) {
      continue; // Skip points outside map bounds
    }

    auto gridIndex = std::make_pair(measuredIndex.x(), measuredIndex.y());
    auto [iter, inserted] = gridCells.try_emplace(gridIndex, point);

    if (inserted) {
      // If new point, get exact grid center position
      iter->second = point;
      map_.getPosition(measuredIndex, measuredPosition);
      iter->second.x = measuredPosition.x();
      iter->second.y = measuredPosition.y();
    } else if (point.z > iter->second.z) {
      // If higher point, update z while keeping grid center position
      iter->second = point;
      map_.getPosition(measuredIndex, measuredPosition);
      iter->second.x = measuredPosition.x();
      iter->second.y = measuredPosition.y();
    }
  }

  auto cloudDownsampled = boost::make_shared<pcl::PointCloud<PointT>>();
  cloudDownsampled->reserve(gridCells.size());
  for (const auto &gridCell : gridCells) {
    cloudDownsampled->points.emplace_back(gridCell.second);
  }

  cloudDownsampled->header = cloud->header;
  return cloudDownsampled;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr
HeightMapper::heightMapping(const typename pcl::PointCloud<PointT>::Ptr &cloud) {

  // 1. Sample pointcloud with max height in each grid cell
  auto griddedCloud =
      griddedFilterWithMaxHeightAlt<PointT>(cloud, cfg_.grid_resolution);

  if (griddedCloud->empty()) {
    return griddedCloud;
  }

  // 2. estimate height
  heightEstimator_->estimate(map_, *griddedCloud);

  return griddedCloud;
}

void HeightMapper::updateMapOrigin(const grid_map::Position &position) {
  map_.move(position);
}

template <typename PointT>
void HeightMapper::raycasting(
    const Eigen::Vector3f &sensorOrigin,
    const typename pcl::PointCloud<PointT>::Ptr &cloud) {
  if (cloud->empty()) {
    return;
  }
  raycaster_.correctHeight(map_, *cloud, sensorOrigin);
}

//////////////////////////////////////////////////
// Explicit instantiation of template functions //
//////////////////////////////////////////////////
// Laser
template void HeightMapper::fastHeightFilter<Laser>(
    const typename pcl::PointCloud<Laser>::Ptr &cloud,
    typename pcl::PointCloud<Laser>::Ptr &filtered_cloud);

template pcl::PointCloud<Laser>::Ptr
HeightMapper::griddedFilterWithMaxHeight<Laser>(
    const pcl::PointCloud<Laser>::Ptr &cloud, float gridSize);

template pcl::PointCloud<Laser>::Ptr
HeightMapper::griddedFilterWithMaxHeightAlt<Laser>(
    const pcl::PointCloud<Laser>::Ptr &cloud, float gridSize);

template typename pcl::PointCloud<Laser>::Ptr
HeightMapper::heightMapping<Laser>(const pcl::PointCloud<Laser>::Ptr &cloud);

template void HeightMapper::raycasting<Laser>(
    const Eigen::Vector3f &sensorOrigin,
    const typename pcl::PointCloud<Laser>::Ptr &cloud);

// Color
template pcl::PointCloud<Color>::Ptr
HeightMapper::griddedFilterWithMaxHeight<Color>(
    const pcl::PointCloud<Color>::Ptr &cloud, float gridSize);

template pcl::PointCloud<Color>::Ptr
HeightMapper::griddedFilterWithMaxHeightAlt<Color>(
    const pcl::PointCloud<Color>::Ptr &cloud, float gridSize);

template void HeightMapper::fastHeightFilter<Color>(
    const typename pcl::PointCloud<Color>::Ptr &cloud,
    typename pcl::PointCloud<Color>::Ptr &filtered_cloud);

template typename pcl::PointCloud<Color>::Ptr
HeightMapper::heightMapping<Color>(const pcl::PointCloud<Color>::Ptr &cloud);

template void HeightMapper::raycasting<Color>(
    const Eigen::Vector3f &sensorOrigin,
    const typename pcl::PointCloud<Color>::Ptr &cloud);
} // namespace height_mapping_ros