/*
 * pcd_conversion.cpp
 *
 *  Created on: Mar 24, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_io/pcd_conversion.h"
#include <pcl_conversions/pcl_conversions.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

namespace height_mapping {

using namespace grid_map;

bool PCDConversion::savePCDFile(const HeightMap &map, const std::string &file_name) {
  try {
    // Convert to pointcloud
    sensor_msgs::PointCloud2 cloud;
    GridMapRosConverter::toPointCloud(map, HeightMap::CoreLayers::ELEVATION, cloud);

    // Convert to PCL pointcloud
    pcl::PCLPointCloud2 pcl_cloud;
    pcl_conversions::toPCL(cloud, pcl_cloud);

    // Save to PCD file
    if (pcl::io::savePCDFile(file_name, pcl_cloud) < 0) {
      throw std::runtime_error("[PCDConversion] Error writing heightmap to " + file_name);
      return false;
    }

    return true;
  } catch (const std::exception &e) {
    throw std::runtime_error("[PCDConversion] Error writing heightmap to " + file_name + ": " +
                             std::string(e.what()));
    return false;
  }
}

bool PCDConversion::fromPCDFile(grid_map::HeightMap &map, const std::string &file_name) {
  // TODO: Implement this
  return true;
}

} // namespace height_mapping
