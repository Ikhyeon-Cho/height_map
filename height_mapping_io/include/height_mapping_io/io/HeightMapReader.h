/*
 * HeightMapReader.h
 *
 *  Created on: Mar 21, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#pragma once

#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

class HeightMapReader {
public:
  HeightMapReader() = default;

  const grid_map::GridMap &openBagFile(const std::string &bagfile,
                                       const std::string &topic);
  const grid_map::GridMap &openPCDFile(const std::string &pcdfile);

  const grid_map::GridMap &getHeightmap() const;

private:
  rosbag::Bag rosbag_;          // rosbag io interface
  grid_map::GridMap heightmap_; // data
};
