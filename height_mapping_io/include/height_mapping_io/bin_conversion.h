/*
 * bin_conversion.h
 *
 *  Created on: Dec 02, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#pragma once

#include <height_mapping_core/height_map/HeightMap.h>

namespace height_mapping {

class BinConversion {
public:
  static bool saveBinFile(const grid_map::HeightMap &map, const std::string &file_name);

  static bool fromBinFile(grid_map::HeightMap &map, const std::string &file_name);
};

} // namespace height_mapping