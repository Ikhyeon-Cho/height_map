/*
 * sensor_processor_node.h
 *
 *  Created on: Nov 25, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <height_mapping_core/height_map/CloudTypes.h>
#include "height_mapping_ros/utils/TransformHandler.h"

namespace height_mapping_ros {

class SensorProcessorNode {
public:
  struct Config {
    std::vector<std::string> inputcloud_topics;
    std::string output_cloud_topic;
    double cloud_publish_rate;
    double downsampling_resolution;
    double min_range_threshold;
    double max_range_threshold;
  } cfg;

  SensorProcessorNode();
  ~SensorProcessorNode() = default;
  void loadConfig(const ros::NodeHandle &nh);

private:
  void initializePubSubs();
  void getProcessingParameters();

  // Synchronized callbacks
  void syncCallback2(const sensor_msgs::PointCloud2ConstPtr &msg1,
                     const sensor_msgs::PointCloud2ConstPtr &msg2);
  void syncCallback3(const sensor_msgs::PointCloud2ConstPtr &msg1,
                     const sensor_msgs::PointCloud2ConstPtr &msg2,
                     const sensor_msgs::PointCloud2ConstPtr &msg3);

  void transformToBaselink(const pcl::PointCloud<Color>::Ptr &cloud,
                           pcl::PointCloud<Color>::Ptr &transformedCloud, const std::string &sensorFrame);

  ros::NodeHandle nh_;

  // Message filters types
  typedef message_filters::Subscriber<sensor_msgs::PointCloud2> CloudSubscriber;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>
      SyncPolicy2;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2,
                                                          sensor_msgs::PointCloud2>
      SyncPolicy3;
  typedef message_filters::Synchronizer<SyncPolicy2> Synchronizer2;
  typedef message_filters::Synchronizer<SyncPolicy3> Synchronizer3;

  // Subscribers and synchronizers
  std::vector<std::shared_ptr<CloudSubscriber>> sub_clouds_;
  std::shared_ptr<Synchronizer2> sync2_;
  std::shared_ptr<Synchronizer3> sync3_;

  // Publisher
  ros::Publisher pub_processed_cloud_;

  // Core implementation
  TransformHandler tf_;
  TransformHandler::FrameID frameID;

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
};
} // namespace height_mapping_ros