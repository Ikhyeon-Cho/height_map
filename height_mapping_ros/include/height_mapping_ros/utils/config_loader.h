#pragma once

#include <ros/node_handle.h>
#include <string>

#include "height_mapping_ros/core/HeightMapper.h"
#include "height_mapping_ros/core/GlobalMapper.h"

namespace height_mapping_ros {

namespace height_mapper {

static HeightMapper::Config loadConfig(const ros::NodeHandle &nh) {

  HeightMapper::Config cfg;
  nh.param<std::string>("height_estimator", cfg.estimator_type, "StatMean");
  nh.param<std::string>("map_frame_id", cfg.frame_id, "map");
  nh.param<double>("map_length_x", cfg.map_length_x, 15.0);
  nh.param<double>("map_length_y", cfg.map_length_y, 15.0);
  nh.param<double>("grid_resolution", cfg.grid_resolution, 0.1);
  nh.param<double>("min_height_threshold", cfg.min_height, -0.2);
  nh.param<double>("max_height_threshold", cfg.max_height, 1.5);

  std::cout << "estimator_type: " << cfg.estimator_type << std::endl;
  std::cout << "frame_id: " << cfg.frame_id << std::endl;
  std::cout << "map_length_x: " << cfg.map_length_x << std::endl;
  std::cout << "map_length_y: " << cfg.map_length_y << std::endl;
  std::cout << "grid_resolution: " << cfg.grid_resolution << std::endl;
  std::cout << "min_height: " << cfg.min_height << std::endl;
  std::cout << "max_height: " << cfg.max_height << std::endl;
  return cfg;
}
} // namespace height_mapper

namespace global_mapper {

static GlobalMapper::Config loadConfig(const ros::NodeHandle &nh) {

  GlobalMapper::Config cfg;
  nh.param<std::string>("estimator_type", cfg.estimator_type, "StatMean");
  nh.param<std::string>("frame_id", cfg.frame_id, "map");
  nh.param<double>("map_length_x", cfg.map_length_x, 400.0);
  nh.param<double>("map_length_y", cfg.map_length_y, 400.0);
  nh.param<double>("grid_resolution", cfg.grid_resolution, 0.1);

  nh.param<std::string>("map_save_dir", cfg.map_save_dir,
                        std::string("/home/") + std::getenv("USER") + "/Downloads");
  return cfg;
}
} // namespace global_mapper

} // namespace height_mapping_ros