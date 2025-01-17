#pragma once

#include <grid_map_ros/grid_map_ros.hpp>
#include <height_mapping_core/height_mapping_core.h>
#include <height_mapping_io/height_mapping_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>

#include "height_mapping_data_collection/KITTIMapWriter.h"
#include "height_mapping_data_collection/KITTIScanWriter.h"
#include <height_mapping_utils/height_mapping_utils.h>

class DataCollectionNode {
public:
  DataCollectionNode();
  ~DataCollectionNode() = default;

private:
  void getNodeParameters();
  void getFrameIDs();
  void setNodeTimers();
  void setupROSInterface();
  void getDataCollectionParameters();

  void initialize();
  void laserCloudCallback(const sensor_msgs::PointCloud2Ptr &msg);
  void updateCurrentHeightMap(grid_map::HeightMap &map,
                              const pcl::PointCloud<Laser>::ConstPtr &cloud);
  bool getSubMap(const grid_map::Position &position, grid_map::GridMap &map);
  bool startMapSaverCallback(std_srvs::Empty::Request &req,
                             std_srvs::Empty::Response &res);
  void dataCollectionTimerCallback(const ros::TimerEvent &event);
  void publishTimerCallback(const ros::TimerEvent &event);

  // ROS members
  ros::NodeHandle nh_;
  ros::NodeHandle nhPriv_{"~"};
  ros::NodeHandle nhFrameID_{nh_, "frame_id"};
  ros::NodeHandle nhDataCollection_{nh_, "data_collection"};

  ros::Subscriber subLidarScan_;

  ros::Publisher pubHeightMap_;
  ros::Publisher pubScan_;

  ros::ServiceServer startCollectionServer_;

  ros::Timer dataCollectionTimer_;
  ros::Timer publishTimer_;

  // Frame IDs
  std::string mapFrame_;
  std::string baselinkFrame_;

  // Core objects
  grid_map::GridMap globalMap_;
  FastHeightFilter::Ptr heightFilter_;
  utils::TransformHandler tf_;
  HeightMapReader mapReader_;
  KITTIScanWriter scanWriter_;
  KITTIMapWriter mapWriter_;

  // Node parameters
  std::string subLidarTopic_;
  std::string dataCollectionPath_{
      "/home/ikhyeon/ros/dev_ws/src/height_mapping/data/"};
  std::string globalMapPath_{
      "/home/ikhyeon/ros/dev_ws/src/height_mapping/maps/globalmap.bag"};

  // Timer parameters
  double dataCollectionPeriod_;
  double publishRate_;

  // Height map parameters
  grid_map::Length mapLength_;
  double minHeightThreshold_;
  double maxHeightThreshold_;

  // Data: pointcloud and grid map
  pcl::PointCloud<Laser>::Ptr processedCloud_;
  grid_map::HeightMap heightMap_;

  // Flags
  bool receivedCloud_{false};
  uint64_t dataCount_{0};
};
