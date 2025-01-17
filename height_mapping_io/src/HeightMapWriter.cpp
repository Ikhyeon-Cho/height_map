/*
 * HeightMapWriter.cpp
 *
 *  Created on: Mar 24, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_io/io/HeightMapWriter.h"

bool HeightMapWriter::writeToBag(const grid_map::GridMap &map,
                                 const std::string &bagfile,
                                 const std::string &topic) {
  try {
    rosbag_.open(bagfile, rosbag::bagmode::Write);

    // Convert to message
    grid_map_msgs::GridMap msg;
    grid_map::GridMapRosConverter::toMessage(map, msg);

    // Write to bag
    rosbag_.write(topic, ros::Time::now(), msg);
    rosbag_.close();

    return true;
  } catch (const std::exception &e) {
    throw std::runtime_error("[HeightMapWriter] Error writing map to bag: " +
                             std::string(e.what()));
    return false;
  }
}

bool HeightMapWriter::writeToPCD(const grid_map::GridMap &map,
                                 const std::string &pcdfile) {
  try {
    // Convert to pointcloud
    sensor_msgs::PointCloud2 cloud;
    grid_map::GridMapRosConverter::toPointCloud(map, "elevation", cloud);

    // Convert to PCL pointcloud
    pcl::PCLPointCloud2 pcl_cloud;
    pcl_conversions::toPCL(cloud, pcl_cloud);

    // Save to PCD file
    if (pcl::io::savePCDFile(pcdfile, pcl_cloud) < 0) {
      throw std::runtime_error("[HeightMapWriter] Failed to save PCD file");
      return false;
    }

    return true;
  } catch (const std::exception &e) {
    throw std::runtime_error("[HeightMapWriter] Error writing map to PCD: " +
                             std::string(e.what()));
    return false;
  }
}
