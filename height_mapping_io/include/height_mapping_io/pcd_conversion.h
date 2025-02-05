/*
 * pcd_conversion.h
 *
 *  Created on: Mar 21, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#pragma once

#include <height_mapping_core/height_map/HeightMap.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

namespace height_mapping {

class PCDConversion {
public:
  static bool savePCDFile(const grid_map::HeightMap &map, const std::string &file_name);

  static bool fromPCDFile(grid_map::HeightMap &map, const std::string &file_name);


};
} // namespace height_mapping
