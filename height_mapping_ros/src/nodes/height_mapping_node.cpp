/*
 * height_mapping_node.cpp
 *
 *  Created on: Nov 24, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_ros/nodes/height_mapping_node.h"
#include "height_mapping_ros/utils/config_loader.h"
#include "height_mapping_ros/utils/pc_utils.h"
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <pcl_conversions/pcl_conversions.h>

namespace height_mapping_ros {

MappingNode::MappingNode() : nh_("~") {

  // ROS node
  ros::NodeHandle nh_node(nh_, "node");
  MappingNode::loadConfig(nh_node);
  initializeTimers();
  initializePubSubs();

  // Mapper object
  ros::NodeHandle nh_mapping(nh_, "mapping");
  auto cfg = height_mapper::loadConfig(nh_mapping);
  mapper_ = std::make_unique<HeightMapper>(cfg);

  // Transform object
  ros::NodeHandle nh_frame_id(nh_, "frame_id");
  frameID = TransformHandler::loadFrameIDs(nh_frame_id);

  std::cout << "\033[1;33m[height_mapping_ros::MappingNode]: "
               "Height mapping node initialized. Waiting for scan inputs...\033[0m\n";
}

void MappingNode::loadConfig(const ros::NodeHandle &nh) {

  // Topic parameters
  cfg_.lidarcloud_topic = nh.param<std::string>("lidar_topic", "/velodyne_points");
  cfg_.rgbdcloud_topic = nh.param<std::string>("rgbd_topic", "/camera/pointcloud/points");

  // Timer parameters
  cfg_.robot_pose_update_rate = nh.param<double>("robot_pose_update_rate", 15.0);
  cfg_.map_publish_rate = nh.param<double>("map_publish_rate", 10.0);

  // Options
  cfg_.remove_backward_points = nh.param<bool>("remove_backward_points", false);
  cfg_.debug_mode = nh.param<bool>("debug_mode", false);
}

void MappingNode::initializeTimers() {

  auto robot_pose_update_duration = ros::Duration(1.0 / cfg_.robot_pose_update_rate);
  auto heightmap_publish_duration = ros::Duration(1.0 / cfg_.map_publish_rate);

  pose_update_timer_ =
      nh_.createTimer(robot_pose_update_duration, &MappingNode::updateMapOrigin, this, false, false);
  map_publish_timer_ =
      nh_.createTimer(heightmap_publish_duration, &MappingNode::publishHeightMap, this, false, false);
}

void MappingNode::initializePubSubs() {

  // Subscribers
  sub_lidarscan_ = nh_.subscribe(cfg_.lidarcloud_topic, 1, &MappingNode::lidarScanCallback, this);
  sub_rgbdscan_ = nh_.subscribe(cfg_.rgbdcloud_topic, 1, &MappingNode::rgbdScanCallback, this);

  // Publishers
  pub_heightmap_ = nh_.advertise<grid_map_msgs::GridMap>("/height_mapping/local/gridmap", 1);
  pub_proc_lidar = nh_.advertise<sensor_msgs::PointCloud2>("/height_mapping/local/lidarcloud", 1);
  pub_proc_rgbd_ = nh_.advertise<sensor_msgs::PointCloud2>("/height_mapping/local/rgbdcloud", 1);

  if (cfg_.debug_mode) {
    pub_debug_lidar_ = nh_.advertise<sensor_msgs::PointCloud2>("debug/lidarcloud", 1);
    pub_debug_rgbd_ = nh_.advertise<sensor_msgs::PointCloud2>("debug/rgbdcloud", 1);
  }
}

void MappingNode::lidarScanCallback(const sensor_msgs::PointCloud2Ptr &msg) {

  // First time receiving pointcloud -> start pose update timer
  if (!lidarscan_received_) {
    lidarscan_received_ = true;
    frameID.sensor = msg->header.frame_id;
    pose_update_timer_.start();
    map_publish_timer_.start();
    std::cout << "\033[1;33m[height_mapping_ros::MappingNode]: Pointcloud Received! "
              << "Use LiDAR scans for height mapping... \033[0m\n";
  }

  // 1. Get transform matrix using tf tree
  geometry_msgs::TransformStamped sensor2base, base2map;
  if (!tf_.lookupTransform(frameID.base_link, frameID.sensor, sensor2base) ||
      !tf_.lookupTransform(frameID.map, frameID.base_link, base2map))
    return;

  // 2. Convert ROS msg to PCL data
  auto scan_raw = boost::make_shared<pcl::PointCloud<Laser>>();
  pcl::moveFromROSMsg(*msg, *scan_raw);

  // 3. Process input scan
  auto scan_preprocessed = processLidarScan(scan_raw, sensor2base, base2map);
  if (!scan_preprocessed)
    return;

  // 4. Height mapping
  auto scan_rasterized = mapper_->heightMapping(scan_preprocessed);

  // 5. Raycasting correction: remove dynamic objects
  auto sensor2map = tf_.combineTransforms(sensor2base, base2map);
  Eigen::Vector3f sensorOrigin3D(sensor2map.transform.translation.x, sensor2map.transform.translation.y,
                                 sensor2map.transform.translation.z);
  mapper_->raycasting(sensorOrigin3D, scan_preprocessed);

  // 6. Publish pointcloud used for mapping
  sensor_msgs::PointCloud2 msg_cloud;
  pcl::toROSMsg(*scan_rasterized, msg_cloud);
  pub_proc_lidar.publish(msg_cloud);

  // Debug: publish pointcloud that you want to see
  if (cfg_.debug_mode) {
    sensor_msgs::PointCloud2 msg_debug;
    pcl::toROSMsg(*scan_preprocessed, msg_debug);
    pub_debug_lidar_.publish(msg_debug);
  }
}

void MappingNode::rgbdScanCallback(const sensor_msgs::PointCloud2Ptr &msg) {

  // First time receiving RGB-D cloud -> start pose update timer
  if (!rgbdscan_received_) {
    rgbdscan_received_ = true;
    pose_update_timer_.start();
    map_publish_timer_.start();
    std::cout << "\033[1;33m[height_mapping_ros::MappingNode]: Colored cloud Received! "
              << "Use RGB-D sensors for height mapping... \033[0m\n";
  }

  // Get Transform matrix
  frameID.sensor = msg->header.frame_id;
  geometry_msgs::TransformStamped camera2base, base2map;
  if (!tf_.lookupTransform(frameID.base_link, frameID.sensor, camera2base) ||
      !tf_.lookupTransform(frameID.map, frameID.base_link, base2map))
    return;

  // Prepare pointcloud
  auto scan_raw = boost::make_shared<pcl::PointCloud<Color>>();
  pcl::moveFromROSMsg(*msg, *scan_raw);

  // Preprocess pointcloud
  auto cloud_processed = processRGBDCloud(scan_raw, camera2base, base2map);
  if (!cloud_processed)
    return;

  // Mapping
  auto cloud_mapped = mapper_->heightMapping<Color>(cloud_processed);

  // Publish pointcloud used for mapping
  sensor_msgs::PointCloud2 cloudMsg;
  pcl::toROSMsg(*cloud_mapped, cloudMsg);
  pub_proc_rgbd_.publish(cloudMsg);

  // Debug: publish pointcloud that you want to see
  if (cfg_.debug_mode) {
    sensor_msgs::PointCloud2 debugMsg;
    pcl::toROSMsg(*cloud_processed, debugMsg);
    pub_debug_rgbd_.publish(debugMsg);
  }
}

pcl::PointCloud<Laser>::Ptr MappingNode::processLidarScan(const pcl::PointCloud<Laser>::Ptr &cloud,
                                                          const geometry_msgs::TransformStamped &lidar2base,
                                                          const geometry_msgs::TransformStamped &base2map) {
  // 1. Filter local pointcloud
  auto cloud_base = pc_utils::applyTransform<Laser>(cloud, lidar2base);
  auto cloud_processed = boost::make_shared<pcl::PointCloud<Laser>>();
  mapper_->fastHeightFilter(cloud_base, cloud_processed);
  auto range = mapper_->getHeightMap().getLength() / 2.0; // mapping range
  cloud_processed = pc_utils::passThrough<Laser>(cloud_processed, "x", -range.x(), range.x());
  cloud_processed = pc_utils::passThrough<Laser>(cloud_processed, "y", -range.y(), range.y());

  // (Optional) Remove remoter points
  if (cfg_.remove_backward_points)
    cloud_processed = pc_utils::filterAngle<Laser>(cloud_processed, -135.0, 135.0);

  // 2. Transform pointcloud to map frame
  cloud_processed = pc_utils::applyTransform<Laser>(cloud_processed, base2map);

  if (cloud_processed->empty())
    return nullptr;
  return cloud_processed;
}

pcl::PointCloud<Color>::Ptr MappingNode::processRGBDCloud(const pcl::PointCloud<Color>::Ptr &cloud,
                                                          const geometry_msgs::TransformStamped &camera2base,
                                                          const geometry_msgs::TransformStamped &base2map) {

  auto cloud_base = pc_utils::applyTransform<Color>(cloud, camera2base);
  auto cloud_processed = boost::make_shared<pcl::PointCloud<Color>>();
  mapper_->fastHeightFilter<Color>(cloud_base, cloud_processed);
  auto range = mapper_->getHeightMap().getLength() / 2.0; // mapping range
  cloud_processed = pc_utils::passThrough<Color>(cloud_processed, "x", -range.x(), range.x());
  cloud_processed = pc_utils::passThrough<Color>(cloud_processed, "y", -range.y(), range.y());
  cloud_processed = pc_utils::applyTransform<Color>(cloud_processed, base2map);

  // (Optional) Remove remoter points
  if (cfg_.remove_backward_points)
    cloud_processed = pc_utils::filterAngle<Color>(cloud_processed, -135.0, 135.0);

  if (cloud_processed->empty())
    return nullptr;
  return cloud_processed;
}

void MappingNode::updateMapOrigin(const ros::TimerEvent &event) {

  geometry_msgs::TransformStamped base2map;
  if (!tf_.lookupTransform(frameID.map, frameID.base_link, base2map))
    return;

  // Update map origin
  auto robot_position =
      grid_map::Position(base2map.transform.translation.x, base2map.transform.translation.y);
  mapper_->moveMapOrigin(robot_position);
}

void MappingNode::publishHeightMap(const ros::TimerEvent &event) {

  grid_map_msgs::GridMap msg;
  grid_map::GridMapRosConverter::toMessage(mapper_->getHeightMap(), msg);
  pub_heightmap_.publish(msg);
}
} // namespace height_mapping_ros

int main(int argc, char **argv) {

  ros::init(argc, argv, "height_mapping_node"); // launch file overrides the name
  height_mapping_ros::MappingNode node;
  ros::spin();

  return 0;
}