/*
 * height_mapping_node.h
 *
 *  Created on: Nov 24, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "height_mapping_ros/core/HeightMapper.h"
#include "height_mapping_ros/utils/TransformHandler.h"
#include <height_mapping_utils/height_mapping_utils.h>

namespace height_mapping_ros {

class MappingNode {
public:
  struct Config {
    std::string lidarcloud_topic;
    std::string rgbdcloud_topic;
    double robot_pose_update_rate;
    double map_publish_rate;
    bool remove_backward_points;
    bool debug_mode;
  };

  MappingNode();
  ~MappingNode() = default;
  void loadConfig(const ros::NodeHandle &nh);

private:
  // init functions
  void initializeTimers();
  void initializeROSCommunication();

  // callback functions -> call core functions
  void lidarScanCallback(const sensor_msgs::PointCloud2Ptr &msg);
  void rgbdScanCallback(const sensor_msgs::PointCloud2Ptr &msg);
  void rayCastingCallback(const sensor_msgs::PointCloud2Ptr &msg);

  // Core functions
  pcl::PointCloud<Laser>::Ptr processLidarScan(const pcl::PointCloud<Laser>::Ptr &cloud);
  void processRGBDCloud();
  void updateMapOrigin(const ros::TimerEvent &event);
  void publishHeightMap(const ros::TimerEvent &event);

  // ROS members
  ros::NodeHandle nh_;                         // "/height_mapping/"
  ros::NodeHandle nhPriv_{"~"};                // "/height_mapping/{node_name}"
  ros::NodeHandle nhMap_{nh_, "height_map"};   // "/height_mapping/height_map/"
  ros::NodeHandle nhFrameID_{nh_, "frame_id"}; // "/height_mapping/frame_id/"

  // Config
  MappingNode::Config cfg_;

  // Subscribers
  ros::Subscriber sub_lidarscan_;
  ros::Subscriber sub_rgbdscan_;
  ros::Subscriber sub_mycloud_; // for raycasting

  // Publishers
  ros::Publisher pub_heightmap_;
  ros::Publisher pub_proc_lidar;
  ros::Publisher pub_proc_rgbd_;
  ros::Publisher pub_debug_lidar_;
  ros::Publisher pub_debug_rgbd_;

  // Timers
  ros::Timer pose_update_timer_;
  ros::Timer map_publish_timer_;

  // Core objects
  std::unique_ptr<HeightMapper> mapper_;
  TransformHandler tf_;
  TransformHandler::FrameID frame_id_;

  // State variables
  bool lidarscan_received_{false};
  bool rgbd_points_received_{false};

  // pointcloud data
  pcl::PointCloud<Laser>::Ptr cloud_for_raycasting_;
};
} // namespace height_mapping_ros