/*
 * HeightMapWriter.h
 *
 *  Created on: Mar 24, 2024
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
#include <sensor_msgs/PointCloud2.h>

class HeightMapWriter {
public:
  HeightMapWriter() = default;

  bool writeToBag(const grid_map::GridMap &map, const std::string &bagfile,
                  const std::string &topic);

  bool writeToPCD(const grid_map::GridMap &map, const std::string &pcdfile);

private:
  rosbag::Bag rosbag_; // rosbag io interface
};
