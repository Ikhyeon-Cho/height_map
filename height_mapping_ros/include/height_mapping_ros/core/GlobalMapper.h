/*
 * GlobalMapper.h
 *
 *  Created on: Dec 2, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#pragma once

#include <height_mapping_core/height_mapping_core.h>
#include <height_mapping_msgs/HeightMapConverter.h>
#include <height_mapping_msgs/HeightMapMsgs.h>

#include <filesystem>
#include <opencv2/opencv.hpp>
#include <unordered_set>

namespace std {
template <> struct hash<grid_map::Index> {
  std::size_t operator()(const grid_map::Index &index) const {
    std::size_t h1 = std::hash<int>{}(index[0]);
    std::size_t h2 = std::hash<int>{}(index[1]);
    return h1 ^ (h2 << 1);
  }
};

template <> struct equal_to<grid_map::Index> {
  bool operator()(const grid_map::Index &lhs,
                  const grid_map::Index &rhs) const {
    return (lhs[0] == rhs[0]) && (lhs[1] == rhs[1]);
  }
};
} // namespace std

class GlobalMapper {
public:
  struct Config {
    std::string estimator_type;
    std::string frame_id;
    std::string map_save_dir;
    double map_length_x;
    double map_length_y;
    double grid_resolution;
  };

  GlobalMapper(const Config &cfg);

  template <typename PointT> void mapping(const pcl::PointCloud<PointT> &cloud);

  void raycasting(const Eigen::Vector3f &sensorOrigin,
                  const pcl::PointCloud<Laser> &cloud);

  void clearMap();

  const grid_map::HeightMap &getHeightMap() const { return globalmap_; }
  const std::unordered_set<grid_map::Index> &getMeasuredGridIndices() const {
    return measured_indices_;
  }

private:
  void initGlobalMap();
  void initHeightEstimator();
  template <typename PointT>
  void updateMeasuredGridIndices(const grid_map::HeightMap &map,
                                 const pcl::PointCloud<PointT> &cloud);

  grid_map::HeightMap globalmap_;
  Config cfg_;

  std::unordered_set<grid_map::Index> measured_indices_;
  height_mapping::HeightEstimatorBase::Ptr heightEstimator_;
  height_mapping::HeightMapRaycaster raycaster_;
};
