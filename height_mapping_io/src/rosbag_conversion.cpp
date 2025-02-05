/*
 * rosbag_conversion.cpp
 *
 *  Created on: Mar 24, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_io/rosbag_conversion.h"
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>

namespace height_mapping {

using namespace grid_map;

bool ROSBagConversion::saveROSBag(const HeightMap &map, const std::string &bagfile, const std::string &topic) {

  try {
    rosbag::Bag bg_api;
    bg_api.open(bagfile, rosbag::bagmode::Write);

    // Convert to message
    grid_map_msgs::GridMap msg;
    grid_map::GridMapRosConverter::toMessage(map, msg);

    // Write to bag
    bg_api.write(topic, ros::Time::now(), msg);
    bg_api.close();

    return true;
  } catch (const std::exception &e) {
    throw std::runtime_error("[ROSBagConversion] Error writing map to bag: " + std::string(e.what()));
    return false;
  }
}

bool ROSBagConversion::fromROSBag(grid_map::HeightMap &map, const std::string &bagfile,
                                  const std::string &topic) {
  // Open bag file
  rosbag::Bag bg_api;
  bg_api.open(bagfile, rosbag::bagmode::Read);

  // Read gridmap message
  rosbag::View view(bg_api, rosbag::TopicQuery(topic));
  if (view.size() == 0) {
    throw std::runtime_error("[ROSBagConversion] No GridMap messages found in "
                             "bagfile!");
    return false;
  }

  // Assume only one GridMap message in the bag file
  auto msg = view.begin()->instantiate<grid_map_msgs::GridMap>();
  bool success = grid_map::GridMapRosConverter::fromMessage(*msg, map);
  if (!success) {
    throw std::runtime_error("[ROSBagConversion] Failed to convert GridMap "
                             "message!");
    return false;
  }

  return true;
}

} // namespace height_mapping
