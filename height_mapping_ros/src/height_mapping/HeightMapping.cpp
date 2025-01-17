/*
 * HeightMapping.cpp
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_ros/HeightMapping.h"

HeightMapping::HeightMapping(const Parameters &params)
    : params_{params}, heightFilter_{params.minHeight, params.maxHeight} {

  paramValidityCheck();

  initHeightMap();

  initHeightEstimator();
}

void HeightMapping::paramValidityCheck() {

  if (params_.gridResolution <= 0) {
    throw std::invalid_argument(
        "[HeightMapping::HeightMapping]: Grid resolution must be positive");
  }
  if (params_.mapLengthX <= 0 || params_.mapLengthY <= 0) {
    throw std::invalid_argument(
        "[HeightMapping::HeightMapping]: Map dimensions must be positive");
  }
}

void HeightMapping::initHeightMap() {
  map_.setFrameId(params_.mapFrame);
  map_.setGeometry(grid_map::Length(params_.mapLengthX, params_.mapLengthY),
                   params_.gridResolution);
}

void HeightMapping::initHeightEstimator() {

  // Set height estimator
  // - Kalman Filter
  // - Moving Average
  // - StatMean (by default)

  if (params_.heightEstimatorType == "KalmanFilter") {
    heightEstimator_ = std::make_unique<height_mapping::KalmanEstimator>();

    std::cout << "\033[1;33m[HeightMapping::HeightMapping]: Height estimator "
                 "type --> KalmanFilter \033[0m\n";

  } else if (params_.heightEstimatorType == "MovingAverage") {
    heightEstimator_ =
        std::make_unique<height_mapping::MovingAverageEstimator>();

    std::cout << "\033[1;33m[HeightMapping::HeightMapping]: Height estimator "
                 "type --> MovingAverage \033[0m\n";

  } else if (params_.heightEstimatorType == "StatMean") {
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
void HeightMapping::fastHeightFilter(
    const typename pcl::PointCloud<PointT>::Ptr &cloud,
    typename pcl::PointCloud<PointT>::Ptr &filtered_cloud) {

  heightFilter_.filter<PointT>(cloud, filtered_cloud);
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr HeightMapping::griddedFilterWithMaxHeight(
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
HeightMapping::griddedFilterWithMaxHeightAlt(
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
HeightMapping::mapping(const typename pcl::PointCloud<PointT>::Ptr &cloud) {

  // 1. Sample pointcloud with max height in each grid cell
  auto griddedCloud =
      griddedFilterWithMaxHeightAlt<PointT>(cloud, params_.gridResolution);

  if (griddedCloud->empty()) {
    return griddedCloud;
  }

  // 2. estimate height
  heightEstimator_->estimate(map_, *griddedCloud);

  return griddedCloud;
}

void HeightMapping::updateMapOrigin(const grid_map::Position &position) {
  map_.move(position);
}

template <typename PointT>
void HeightMapping::raycasting(
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
template void HeightMapping::fastHeightFilter<Laser>(
    const typename pcl::PointCloud<Laser>::Ptr &cloud,
    typename pcl::PointCloud<Laser>::Ptr &filtered_cloud);

template pcl::PointCloud<Laser>::Ptr
HeightMapping::griddedFilterWithMaxHeight<Laser>(
    const pcl::PointCloud<Laser>::Ptr &cloud, float gridSize);

template pcl::PointCloud<Laser>::Ptr
HeightMapping::griddedFilterWithMaxHeightAlt<Laser>(
    const pcl::PointCloud<Laser>::Ptr &cloud, float gridSize);

template typename pcl::PointCloud<Laser>::Ptr
HeightMapping::mapping<Laser>(const pcl::PointCloud<Laser>::Ptr &cloud);

template void HeightMapping::raycasting<Laser>(
    const Eigen::Vector3f &sensorOrigin,
    const typename pcl::PointCloud<Laser>::Ptr &cloud);

// Color
template pcl::PointCloud<Color>::Ptr
HeightMapping::griddedFilterWithMaxHeight<Color>(
    const pcl::PointCloud<Color>::Ptr &cloud, float gridSize);

template pcl::PointCloud<Color>::Ptr
HeightMapping::griddedFilterWithMaxHeightAlt<Color>(
    const pcl::PointCloud<Color>::Ptr &cloud, float gridSize);

template void HeightMapping::fastHeightFilter<Color>(
    const typename pcl::PointCloud<Color>::Ptr &cloud,
    typename pcl::PointCloud<Color>::Ptr &filtered_cloud);

template typename pcl::PointCloud<Color>::Ptr
HeightMapping::mapping<Color>(const pcl::PointCloud<Color>::Ptr &cloud);

template void HeightMapping::raycasting<Color>(
    const Eigen::Vector3f &sensorOrigin,
    const typename pcl::PointCloud<Color>::Ptr &cloud);
