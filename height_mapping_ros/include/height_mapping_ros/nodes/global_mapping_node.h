/*
 * global_mapping_node.h
 *
 *  Created on: Nov 24, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include "height_mapping_ros/core/GlobalMapper.h"
#include "height_mapping_ros/utils/TransformHandler.h"
#include <height_mapping_io/height_mapping_io.h>

namespace height_mapping_ros {

class GlobalMappingNode {
public:
  struct Config {
    std::string lidarcloud_topic;
    std::string rgbdcloud_topic;
    double map_publish_rate;
    bool remove_backward_points;
    bool debug_mode;
    std::string map_save_dir;
  };

  GlobalMappingNode();
  ~GlobalMappingNode() = default;
  void loadConfig(const ros::NodeHandle &nh);

private:
  // init functions
  void initializeTimers();
  void initializePubSubs();
  void initializeServices();

  // callback functions
  void lidarScanCallback(const sensor_msgs::PointCloud2Ptr &msg);
  void rgbdScanCallback(const sensor_msgs::PointCloud2Ptr &msg);

  // Core functions
  pcl::PointCloud<Laser>::Ptr processLidarScan(const pcl::PointCloud<Laser>::Ptr &cloud,
                                               const geometry_msgs::TransformStamped &lidar2base,
                                               const geometry_msgs::TransformStamped &base2map);
  pcl::PointCloud<Color>::Ptr processRGBDCloud(const pcl::PointCloud<Color>::Ptr &cloud,
                                               const geometry_msgs::TransformStamped &camera2base,
                                               const geometry_msgs::TransformStamped &base2map);

  bool saveMapCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool clearMapCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  // Helper functions
  void publishPointCloudMap(const ros::TimerEvent &event);
  void toPointCloud2(const grid_map::HeightMap &map, const std::vector<std::string> &layers,
                     const std::unordered_set<grid_map::Index> &grid_indices,
                     sensor_msgs::PointCloud2 &cloud);
  void toMapRegion(const grid_map::HeightMap &map, visualization_msgs::Marker &region);

  ros::NodeHandle nh_;

  // Config
  GlobalMappingNode::Config cfg_;

  // Subscribers
  ros::Subscriber sub_lidarscan_;
  ros::Subscriber sub_rgbdscan_;

  // Publishers
  ros::Publisher pub_map_cloud_;
  ros::Publisher pub_map_region_;
  ros::Publisher pub_scan_rasterized_;

  // Services
  ros::ServiceServer srv_clear_map_;
  ros::ServiceServer srv_save_map_;

  // Timers
  ros::Timer map_publish_timer_;

  // Core mapping object
  std::unique_ptr<GlobalMapper> mapper_;
  TransformHandler tf_;
  TransformHandler::FrameID frameID;

  // Map writer
  HeightMapWriter mapWriter_;

  // State variables
  bool lidarscan_received_{false};
  bool rgbdscan_received_{false};
};

} // namespace height_mapping_ros
