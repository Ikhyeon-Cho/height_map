/*
 * HeightMapReader.cpp
 *
 *  Created on: Mar 21, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_io/io/HeightMapReader.h"

const grid_map::GridMap &
HeightMapReader::openBagFile(const std::string &bagfile,
                             const std::string &topic) {
  // Open bag file
  rosbag_.open(bagfile, rosbag::bagmode::Read);

  // Read gridmap message
  rosbag::View view(rosbag_, rosbag::TopicQuery(topic));
  if (view.size() == 0)
    throw std::runtime_error("[HeightMapReader] No GridMap messages found in "
                             "bagfile!");

  // Assume only one GridMap message in the bag file
  auto msg = view.begin()->instantiate<grid_map_msgs::GridMap>();
  bool success = grid_map::GridMapRosConverter::fromMessage(*msg, heightmap_);
  if (!success)
    throw std::runtime_error("[HeightMapReader] Failed to convert GridMap "
                             "message!");

  return heightmap_;
}

const grid_map::GridMap &
HeightMapReader::openPCDFile(const std::string &pcdfile) {
  // TODO: Implement this
  return heightmap_;
}

const grid_map::GridMap &HeightMapReader::getHeightmap() const {
  return heightmap_;
}
