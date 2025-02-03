/*
 * sensor_processor_node.h
 *
 *  Created on: Nov 25, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#pragma once

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <height_mapping_core/height_mapping_core.h>
#include <height_mapping_utils/height_mapping_utils.h>

class SensorProcessorNode {
public:
  SensorProcessorNode();
  ~SensorProcessorNode() = default;

private:
  void getNodeParameters();
  void getFrameIDs();
  void setupROSInterface();
  void getProcessingParameters();

  // Synchronized callbacks
  void syncCallback2(const sensor_msgs::PointCloud2ConstPtr &msg1,
                     const sensor_msgs::PointCloud2ConstPtr &msg2);
  void syncCallback3(const sensor_msgs::PointCloud2ConstPtr &msg1,
                     const sensor_msgs::PointCloud2ConstPtr &msg2,
                     const sensor_msgs::PointCloud2ConstPtr &msg3);

  void transformToBaselink(const pcl::PointCloud<Color>::Ptr &cloud,
                           pcl::PointCloud<Color>::Ptr &transformedCloud,
                           const std::string &sensorFrame);

  // ROS members
  ros::NodeHandle nh_;
  ros::NodeHandle nhPriv_{"~"};
  ros::NodeHandle nhFrameID_{nh_, "frame_id"};
  ros::NodeHandle nhSensorProcessor_{nh_, "sensor_processor"};

  // Frame IDs
  std::string baselinkFrame_;

  // Message filters types
  typedef message_filters::Subscriber<sensor_msgs::PointCloud2> CloudSubscriber;
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>
      SyncPolicy2;
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2, sensor_msgs::PointCloud2,
      sensor_msgs::PointCloud2>
      SyncPolicy3;
  typedef message_filters::Synchronizer<SyncPolicy2> Synchronizer2;
  typedef message_filters::Synchronizer<SyncPolicy3> Synchronizer3;

  // Subscribers and synchronizers
  std::vector<std::shared_ptr<CloudSubscriber>> cloudSubs_;
  std::shared_ptr<Synchronizer2> sync2_;
  std::shared_ptr<Synchronizer3> sync3_;

  // Publisher
  ros::Publisher pubProcessedCloud_;

  // Timer
  ros::Timer cloudPublishTimer_;

  // Core implementation
  // std::unique_ptr<SensorProcessor> sensorProcessor_;
  utils::TransformHandler tf_;

  // pcl pointers
  pcl::PointCloud<Color>::Ptr cloud1_{new pcl::PointCloud<Color>};
  pcl::PointCloud<Color>::Ptr cloud2_{new pcl::PointCloud<Color>};
  pcl::PointCloud<Color>::Ptr cloud3_{new pcl::PointCloud<Color>};

  pcl::PointCloud<Color>::Ptr downsampled1_{new pcl::PointCloud<Color>};
  pcl::PointCloud<Color>::Ptr downsampled2_{new pcl::PointCloud<Color>};
  pcl::PointCloud<Color>::Ptr downsampled3_{new pcl::PointCloud<Color>};

  pcl::PointCloud<Color>::Ptr filtered1_{new pcl::PointCloud<Color>};
  pcl::PointCloud<Color>::Ptr filtered2_{new pcl::PointCloud<Color>};
  pcl::PointCloud<Color>::Ptr filtered3_{new pcl::PointCloud<Color>};

  pcl::PointCloud<Color>::Ptr transformed1_{new pcl::PointCloud<Color>};
  pcl::PointCloud<Color>::Ptr transformed2_{new pcl::PointCloud<Color>};
  pcl::PointCloud<Color>::Ptr transformed3_{new pcl::PointCloud<Color>};

  // Parameters
  std::vector<std::string> inputTopics_;
  std::string outputCloudTopic_;

  double cloudPublishRate_;
  double downsamplingResolution_;
  double minRangeThreshold_;
  double maxRangeThreshold_;

  // Declare point cloud members
};
