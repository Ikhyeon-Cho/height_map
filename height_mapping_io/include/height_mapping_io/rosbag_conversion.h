/*
 * rosbag_conversion.h
 *
 *  Created on: Mar 21, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#pragma once

#include <height_mapping_core/height_map/HeightMap.h>

namespace height_mapping {

class ROSBagConversion {
public:
  static bool saveROSBag(const grid_map::HeightMap &map, const std::string &bagfile, const std::string &topic);

  static bool fromROSBag(grid_map::HeightMap &map, const std::string &bagfile, const std::string &topic);
};
} // namespace height_mapping