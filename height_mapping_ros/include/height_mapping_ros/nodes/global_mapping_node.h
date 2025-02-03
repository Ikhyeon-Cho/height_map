/*
 * global_mapping_node.h
 *
 *  Created on: Nov 24, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#pragma once

#include <grid_map_cv/GridMapCvConverter.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>

#include "height_mapping_ros/core/GlobalMapper.h"
#include <height_mapping_io/height_mapping_io.h>
#include <height_mapping_utils/height_mapping_utils.h>

class GlobalMappingNode {
public:
  GlobalMappingNode();
  ~GlobalMappingNode() = default;

private:
  void getNodeParameters();
  void getFrameIDs();
  void initializeTimers();
  void setupROSInterface();
  GlobalMapper::Config getGlobalMappingParameters();

  void laserCloudCallback(const sensor_msgs::PointCloud2Ptr &msg);
  void rgbCloudCallback(const sensor_msgs::PointCloud2Ptr &msg);
  void publishMap(const ros::TimerEvent &event);

  bool saveMapCallback(std_srvs::Empty::Request &req,
                       std_srvs::Empty::Response &res);
  bool clearMapCallback(std_srvs::Empty::Request &req,
                        std_srvs::Empty::Response &res);
  // bool saveMapCallback(height_mapping_msgs::SaveLayerToImage::Request &req,
  //                      height_mapping_msgs::SaveLayerToImage::Response &res);

  void toPointCloud2(const grid_map::HeightMap &map,
                     const std::vector<std::string> &layers,
                     const std::unordered_set<grid_map::Index> &grid_indices,
                     sensor_msgs::PointCloud2 &cloud);

  // ROS members
  ros::NodeHandle nh_;                // "/height_mapping/"
  ros::NodeHandle nhPriv_{"~"};       // "/height_mapping/global_mapping"
  ros::NodeHandle nhMap_{nh_, "height_map"}; // "/height_mapping/height_map/"
  ros::NodeHandle nhGlobalMap_{nh_, "global_map"};
  ros::NodeHandle nhFrameID_{nh_, "frame_id"};

  // Subscribers
  ros::Subscriber subLaserCloud_;
  ros::Subscriber subRGBCloud_;

  // Publishers
  ros::Publisher pubGlobalMap_;
  ros::Publisher pubMapRegion_;

  // Services
  ros::ServiceServer srvClearMap_;
  ros::ServiceServer srvSaveMapToBag_;

  // Timers
  ros::Timer mapPublishTimer_;

  // Frame IDs
  std::string mapFrame_;
  std::string baselinkFrame_;
  std::string lidarFrame_;

  // Parameters
  bool debugMode_{false};
  double mapPublishRate_;
  std::string mapSavePath_;

  // Core mapping object
  std::unique_ptr<GlobalMapper> globalMapper_;
  utils::TransformHandler tf_;
  HeightMapWriter mapWriter_;

  // State variables
  bool laserReceived_{false};
  bool rgbReceived_{false};
};