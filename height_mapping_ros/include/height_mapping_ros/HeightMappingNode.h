/*
 * HeightMappingNode.h
 *
 *  Created on: Nov 24, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#pragma once

#include <grid_map_ros/GridMapRosConverter.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <height_mapping_utils/height_mapping_utils.h>
#include "height_mapping_ros/HeightMapping.h"

class HeightMappingNode {
public:
  HeightMappingNode();
  ~HeightMappingNode() = default;

private:
  void getNodeParameters();
  void getFrameIDs();
  void setNodeTimers();
  HeightMapping::Parameters getHeightMappingParameters();
  void setupROSInterface();

  void laserCloudCallback(const sensor_msgs::PointCloud2Ptr &msg);
  void rgbCloudCallback(const sensor_msgs::PointCloud2Ptr &msg);
  void updateMapOrigin(const ros::TimerEvent &event);
  void publishMap(const ros::TimerEvent &event);

  // ROS members
  ros::NodeHandle nh_;                         // "/height_mapping/"
  ros::NodeHandle nhPriv_{"~"};                // "/height_mapping/{node_name}"
  ros::NodeHandle nhMap_{nh_, "height_map"};          // "/height_mapping/height_map/"
  ros::NodeHandle nhFrameID_{nh_, "frame_id"}; // "/height_mapping/frame_id/"

  // Frame IDs
  std::string mapFrame_;
  std::string baselinkFrame_;

  // Subscribers
  ros::Subscriber subLaserCloud_;
  ros::Subscriber subRGBCloud_;

  // Publishers
  ros::Publisher pubHeightMap_;
  ros::Publisher pubLaserProcessed_;
  ros::Publisher pubRGBProcessed_;
  ros::Publisher pubDebugLaserCloud_;
  ros::Publisher pubDebugRGBCloud_;

  // Timers
  ros::Timer robotPoseUpdateTimer_;
  ros::Timer mapPublishTimer_;

  // Core height mapping implementation
  std::unique_ptr<HeightMapping> heightMapping_;
  utils::TransformHandler tf_;

  // Parameters
  std::string subLaserTopic_;
  std::string subRGBTopic_;
  double robotPoseUpdateRate_;
  double mapPublishRate_;
  bool debugMode_;

  // State variables
  bool laserReceived_{false};
  bool rgbReceived_{false};
  bool useLidar_{true};
  bool useRGBD_{true};
  bool removeRemoterPoints_{true};
};
