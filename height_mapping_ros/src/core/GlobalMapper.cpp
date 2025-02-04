/*
 * GlobalMapper.cpp
 *
 *  Created on: Dec 2, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_ros/core/GlobalMapper.h"

GlobalMapper::GlobalMapper(const Config &cfg) : cfg_(cfg) {

  initMap();
  initHeightEstimator();

  measured_indices_.reserve(map_.getSize().prod());
}

void GlobalMapper::initMap() {

  // Check parameter validity
  if (cfg_.grid_resolution <= 0) {
    throw std::invalid_argument("[GlobalMapper::initMap]: Grid resolution must be positive");
  }
  if (cfg_.map_length_x <= 0 || cfg_.map_length_y <= 0) {
    throw std::invalid_argument("[GlobalMapper::initMap]: Map dimensions must be positive");
  }

  // Initialize map geometry
  map_.setFrameId(cfg_.frame_id);
  map_.setPosition(grid_map::Position(0.0, 0.0));
  map_.setGeometry(grid_map::Length(cfg_.map_length_x, cfg_.map_length_y), cfg_.grid_resolution);
}

void GlobalMapper::initHeightEstimator() {

  if (cfg_.estimator_type == "KalmanFilter") {
    height_estimator_ = std::make_unique<height_mapping::KalmanEstimator>();
  } else if (cfg_.estimator_type == "MovingAverage") {
    height_estimator_ = std::make_unique<height_mapping::MovingAverageEstimator>();
  } else if (cfg_.estimator_type == "StatMean") {
    height_estimator_ = std::make_unique<height_mapping::StatMeanEstimator>();
  } else {
    height_estimator_ = std::make_unique<height_mapping::StatMeanEstimator>();
  }
}

template <typename PointT> void GlobalMapper::mapping(const pcl::PointCloud<PointT> &cloud) {

  recordMeasuredCells(map_, cloud);

  height_estimator_->estimate(map_, cloud);
}

// Save measured indices for efficiency
template <typename PointT>
void GlobalMapper::recordMeasuredCells(const grid_map::HeightMap &map, const pcl::PointCloud<PointT> &cloud) {

  grid_map::Index cell_index;
  grid_map::Position cell_position;

  for (const auto &point : cloud.points) {
    cell_position = grid_map::Position(point.x, point.y);

    // Skip if the point is out of the map
    if (!map.getIndex(cell_position, cell_index))
      continue;
    if (!map.isEmptyAt(cell_index))
      continue;

    measured_indices_.insert(cell_index);
  }
}

void GlobalMapper::raycasting(const Eigen::Vector3f &sensorOrigin, const pcl::PointCloud<Laser> &cloud) {
  raycaster_.correctHeight(map_, cloud, sensorOrigin);
}

void GlobalMapper::clearMap() { map_.clearAll(); }

//////////////////////////////////////////////////
// Explicit instantiation of template functions //
//////////////////////////////////////////////////
// Laser
template void GlobalMapper::mapping<Laser>(const pcl::PointCloud<Laser> &cloud);
template void GlobalMapper::recordMeasuredCells(const grid_map::HeightMap &map,
                                                const pcl::PointCloud<Laser> &cloud);

// Color
template void GlobalMapper::mapping<Color>(const pcl::PointCloud<Color> &cloud);
template void GlobalMapper::recordMeasuredCells(const grid_map::HeightMap &map,
                                                const pcl::PointCloud<Color> &cloud);
