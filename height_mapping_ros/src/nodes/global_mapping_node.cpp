/*
 * global_mapping_node.cpp
 *
 *  Created on: Nov 24, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_ros/nodes/global_mapping_node.h"
#include "height_mapping_ros/utils/config_loader.h"
#include "height_mapping_ros/utils/pc_utils.h"

#include <sensor_msgs/point_cloud2_iterator.h>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rosbag/bag.h>
#include <visualization_msgs/Marker.h>
#include <filesystem>

namespace height_mapping_ros {

GlobalMappingNode::GlobalMappingNode() : nh_("~") {

  // ROS node
  ros::NodeHandle nh_node(nh_, "node");
  GlobalMappingNode::loadConfig(nh_node);
  initializeTimers();
  initializePubSubs();
  initializeServices();

  // Mapper object
  ros::NodeHandle nh_mapper(nh_, "mapper");
  auto cfg = global_mapper::loadConfig(nh_mapper);
  mapper_ = std::make_unique<GlobalMapper>(cfg);

  // Transform object
  ros::NodeHandle nh_frame_id(nh_, "frame_id");
  frameID = TransformHandler::loadFrameIDs(nh_frame_id);

  std::cout << "\033[1;33m[height_mapping_ros::GlobalMappingNode]: "
               "Global mapping node initialized. Waiting for scan inputs...\033[0m\n";
}

void GlobalMappingNode::loadConfig(const ros::NodeHandle &nh) {

  // Topic parameters
  cfg_.lidarcloud_topic = nh.param<std::string>("lidar_topic", "/velodyne_points");
  cfg_.rgbdcloud_topic = nh.param<std::string>("rgbd_topic", "/camera/pointcloud/points");

  // Timer parameters
  cfg_.map_publish_rate = nh.param<double>("map_publish_rate", 10.0);

  // Options
  cfg_.remove_backward_points = nh.param<bool>("remove_backward_points", false);
  cfg_.debug_mode = nh.param<bool>("debug_mode", false);
}

void GlobalMappingNode::initializeTimers() {

  auto map_publish_duration = ros::Duration(1.0 / cfg_.map_publish_rate);

  map_publish_timer_ =
      nh_.createTimer(map_publish_duration, &GlobalMappingNode::publishPointCloudMap, this, false, false);
}

void GlobalMappingNode::initializePubSubs() {

  // Subscribers
  sub_lidarscan_ = nh_.subscribe(cfg_.lidarcloud_topic, 1, &GlobalMappingNode::lidarScanCallback, this);
  sub_rgbdscan_ = nh_.subscribe(cfg_.rgbdcloud_topic, 1, &GlobalMappingNode::rgbdScanCallback, this);

  // Publishers
  pub_map_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/height_mapping/global/map_cloud", 1);
  pub_map_region_ = nh_.advertise<visualization_msgs::Marker>("/height_mapping/global/map_region", 1);
  pub_scan_rasterized_ = nh_.advertise<sensor_msgs::PointCloud2>("/height_mapping/global/scan_rasterized", 1);
}

void GlobalMappingNode::initializeServices() {

  srv_save_map_ =
      nh_.advertiseService("/height_mapping/global/save_map", &GlobalMappingNode::saveMapCallback, this);
  srv_clear_map_ =
      nh_.advertiseService("/height_mapping/global/clear_map", &GlobalMappingNode::clearMapCallback, this);
}

void GlobalMappingNode::lidarScanCallback(const sensor_msgs::PointCloud2Ptr &msg) {

  if (!lidarscan_received_) {
    lidarscan_received_ = true;
    frameID.sensor = msg->header.frame_id;
    map_publish_timer_.start();
    std::cout << "\033[1;32m[height_mapping_ros::GlobalMappingNode]: Pointcloud Received! "
              << "Use LiDAR scans for global mapping... \033[0m\n";
  }

  // 1. Get transform matrix using tf tree
  geometry_msgs::TransformStamped sensor2base, base2map;
  if (!tf_.lookupTransform(frameID.base_link, frameID.sensor, sensor2base) ||
      !tf_.lookupTransform(frameID.map, frameID.base_link, base2map))
    return;

  // 2. Convert ROS msg to PCL data
  auto scan_raw = boost::make_shared<pcl::PointCloud<Laser>>();
  pcl::moveFromROSMsg(*msg, *scan_raw);

  auto scan_processed = processLidarScan(scan_raw, sensor2base, base2map);
  if (!scan_processed)
    return;

  auto scan_rasterized = mapper_->heightMapping(scan_processed);

  auto sensor2map = tf_.combineTransforms(sensor2base, base2map);
  Eigen::Vector3f sensorOrigin3D(sensor2map.transform.translation.x, sensor2map.transform.translation.y,
                                 sensor2map.transform.translation.z);
  mapper_->raycasting(sensorOrigin3D, scan_processed);

  sensor_msgs::PointCloud2 msg_cloud;
  pcl::toROSMsg(*scan_rasterized, msg_cloud);
  pub_scan_rasterized_.publish(msg_cloud);
}

void GlobalMappingNode::rgbdScanCallback(const sensor_msgs::PointCloud2Ptr &msg) {

  if (!rgbdscan_received_) {
    rgbdscan_received_ = true;
    frameID.sensor = msg->header.frame_id;
    map_publish_timer_.start();
    std::cout << "\033[1;32m[height_mapping_ros::GlobalMappingNode]: Colored cloud received! "
              << "Start global mapping... \033[0m\n";
  }

  // 1. Get transform matrix using tf tree
  geometry_msgs::TransformStamped sensor2base, base2map;
  if (!tf_.lookupTransform(frameID.base_link, frameID.sensor, sensor2base) ||
      !tf_.lookupTransform(frameID.map, frameID.base_link, base2map))
    return;

  // 2. Convert ROS msg to PCL data
  auto scan_raw = boost::make_shared<pcl::PointCloud<Color>>();
  pcl::moveFromROSMsg(*msg, *scan_raw);

  auto scan_processed = processRGBDCloud(scan_raw, sensor2base, base2map);
  if (!scan_processed)
    return;

  auto scan_rasterized = mapper_->heightMapping(scan_processed);

  auto sensor2map = tf_.combineTransforms(sensor2base, base2map);
  Eigen::Vector3f sensorOrigin3D(sensor2map.transform.translation.x, sensor2map.transform.translation.y,
                                 sensor2map.transform.translation.z);
  mapper_->raycasting(sensorOrigin3D, scan_processed);

  sensor_msgs::PointCloud2 msg_cloud;
  pcl::toROSMsg(*scan_rasterized, msg_cloud);
  pub_scan_rasterized_.publish(msg_cloud);
}

pcl::PointCloud<Laser>::Ptr
GlobalMappingNode::processLidarScan(const pcl::PointCloud<Laser>::Ptr &cloud,
                                    const geometry_msgs::TransformStamped &lidar2base,
                                    const geometry_msgs::TransformStamped &base2map) {
  // 1. Filter local pointcloud
  auto cloud_base = pc_utils::applyTransform<Laser>(cloud, lidar2base);
  auto cloud_processed = boost::make_shared<pcl::PointCloud<Laser>>();
  mapper_->fastHeightFilter(cloud_base, cloud_processed);
  cloud_processed = pc_utils::passThrough<Laser>(cloud_processed, "x", -10.0, 10.0);
  cloud_processed = pc_utils::passThrough<Laser>(cloud_processed, "y", -10.0, 10.0);

  // (Optional) Remove remoter points
  if (cfg_.remove_backward_points)
    cloud_processed = pc_utils::filterAngle<Laser>(cloud_processed, -135.0, 135.0);

  // 2. Transform pointcloud to map frame
  cloud_processed = pc_utils::applyTransform<Laser>(cloud_processed, base2map);

  if (cloud_processed->empty())
    return nullptr;
  return cloud_processed;
}

pcl::PointCloud<Color>::Ptr
GlobalMappingNode::processRGBDCloud(const pcl::PointCloud<Color>::Ptr &cloud,
                                    const geometry_msgs::TransformStamped &camera2base,
                                    const geometry_msgs::TransformStamped &base2map) {

  auto cloud_base = pc_utils::applyTransform<Color>(cloud, camera2base);
  auto cloud_processed = boost::make_shared<pcl::PointCloud<Color>>();
  mapper_->fastHeightFilter(cloud_base, cloud_processed);
  cloud_processed = pc_utils::passThrough<Color>(cloud_processed, "x", -10.0, 10.0);
  cloud_processed = pc_utils::passThrough<Color>(cloud_processed, "y", -10.0, 10.0);

  // (Optional) Remove remoter points
  if (cfg_.remove_backward_points)
    cloud_processed = pc_utils::filterAngle<Color>(cloud_processed, -135.0, 135.0);

  cloud_processed = pc_utils::applyTransform<Color>(cloud_processed, base2map);

  if (cloud_processed->empty())
    return nullptr;
  return cloud_processed;
}

void GlobalMappingNode::publishPointCloudMap(const ros::TimerEvent &) {

  std::vector<std::string> layers = {
      grid_map::HeightMap::CoreLayers::ELEVATION,      grid_map::HeightMap::CoreLayers::ELEVATION_MAX,
      grid_map::HeightMap::CoreLayers::ELEVATION_MIN,  grid_map::HeightMap::CoreLayers::VARIANCE,
      grid_map::HeightMap::CoreLayers::N_MEASUREMENTS,

  };
  // Visualize global map
  sensor_msgs::PointCloud2 msg_map_cloud;
  toPointCloud2(mapper_->getHeightMap(), layers, mapper_->getMeasuredGridIndices(), msg_map_cloud);
  pub_map_cloud_.publish(msg_map_cloud);

  // Visualize map region
  visualization_msgs::Marker msg_map_region;
  toMapRegion(mapper_->getHeightMap(), msg_map_region);
  pub_map_region_.publish(msg_map_region);
}

void GlobalMappingNode::toPointCloud2(const grid_map::HeightMap &map, const std::vector<std::string> &layers,
                                      const std::unordered_set<grid_map::Index> &measuredIndices,
                                      sensor_msgs::PointCloud2 &cloud) {

  // Setup cloud header
  cloud.header.frame_id = map.getFrameId();
  cloud.header.stamp.fromNSec(map.getTimestamp());
  cloud.is_dense = false;

  // Setup field names and cloud structure
  std::vector<std::string> fieldNames;
  fieldNames.reserve(layers.size());

  // Setup field names
  fieldNames.insert(fieldNames.end(), {"x", "y", "z"});
  for (const auto &layer : layers) {
    if (layer == "color") {
      fieldNames.push_back("rgb");
    } else {
      fieldNames.push_back(layer);
    }
  }

  // Setup point field structure
  cloud.fields.clear();
  cloud.fields.reserve(fieldNames.size());
  int offset = 0;

  for (const auto &name : fieldNames) {
    sensor_msgs::PointField field;
    field.name = name;
    field.count = 1;
    field.datatype = sensor_msgs::PointField::FLOAT32;
    field.offset = offset;
    cloud.fields.push_back(field);
    offset += sizeof(float);
  }

  // Initialize cloud size
  const size_t num_points = measuredIndices.size();
  cloud.height = 1;
  cloud.width = num_points;
  cloud.point_step = offset;
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.data.resize(cloud.height * cloud.row_step);

  // Setup point field iterators
  std::unordered_map<std::string, sensor_msgs::PointCloud2Iterator<float>> iterators;
  for (const auto &name : fieldNames) {
    iterators.emplace(name, sensor_msgs::PointCloud2Iterator<float>(cloud, name));
  }

  // Fill point cloud data
  size_t validPoints = 0;
  for (const auto &index : measuredIndices) {
    grid_map::Position3 position;
    if (!map.getPosition3(grid_map::HeightMap::CoreLayers::ELEVATION, index, position)) {
      continue;
    }

    // Update each field
    for (auto &[fieldName, iterator] : iterators) {
      if (fieldName == "x")
        *iterator = static_cast<float>(position.x());
      else if (fieldName == "y")
        *iterator = static_cast<float>(position.y());
      else if (fieldName == "z")
        *iterator = static_cast<float>(position.z());
      else if (fieldName == "rgb")
        *iterator = static_cast<float>(map.at("color", index));
      else
        *iterator = static_cast<float>(map.at(fieldName, index));
      ++iterator;
    }
    ++validPoints;
  }

  // Adjust final cloud size
  cloud.width = validPoints;
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.data.resize(cloud.height * cloud.row_step);
}

void GlobalMappingNode::toMapRegion(const grid_map::HeightMap &map, visualization_msgs::Marker &marker) {

  marker.ns = "height_map";
  marker.lifetime = ros::Duration();
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::LINE_STRIP;

  marker.scale.x = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  marker.header.frame_id = map.getFrameId();
  marker.header.stamp = ros::Time::now();

  float length_x_half = (map.getLength().x() - 0.5 * map.getResolution()) / 2.0;
  float length_y_half = (map.getLength().y() - 0.5 * map.getResolution()) / 2.0;

  marker.points.resize(5);
  marker.points[0].x = map.getPosition().x() + length_x_half;
  marker.points[0].y = map.getPosition().y() + length_x_half;
  marker.points[0].z = 0;

  marker.points[1].x = map.getPosition().x() + length_x_half;
  marker.points[1].y = map.getPosition().y() - length_x_half;
  marker.points[1].z = 0;

  marker.points[2].x = map.getPosition().x() - length_x_half;
  marker.points[2].y = map.getPosition().y() - length_x_half;
  marker.points[2].z = 0;

  marker.points[3].x = map.getPosition().x() - length_x_half;
  marker.points[3].y = map.getPosition().y() + length_x_half;
  marker.points[3].z = 0;

  marker.points[4] = marker.points[0];
}

bool GlobalMappingNode::clearMapCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {

  mapper_->clearMap();
  return true;
}

bool GlobalMappingNode::saveMapCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  try {
    // Folder check and creation
    std::filesystem::path save_path(cfg_.map_save_dir);
    std::filesystem::path save_dir = save_path.has_extension() ? save_path.parent_path() : save_path;

    if (!std::filesystem::exists(save_dir)) {
      std::filesystem::create_directories(save_dir);
    }

    // Save GridMap to bag
    mapWriter_.writeToBag(mapper_->getHeightMap(), cfg_.map_save_dir, "/height_mapping/globalmap/gridmap");

    // Save GridMap to PCD
    mapWriter_.writeToPCD(mapper_->getHeightMap(),
                          cfg_.map_save_dir.substr(0, cfg_.map_save_dir.rfind('.')) + ".pcd");

    std::cout << "\033[1;33m[height_mapping_ros::GlobalMappingNode]: Successfully saved "
              << "map to " << cfg_.map_save_dir << "\033[0m\n";

  } catch (const std::exception &e) {
    std::cout << "\033[1;31m[height_mapping_ros::GlobalMappingNode]: Failed to save map: "
              << std::string(e.what()) << "\033[0m\n";
  }

  return true;
}

} // namespace height_mapping_ros

int main(int argc, char **argv) {

  ros::init(argc, argv, "global_mapping_node");
  height_mapping_ros::GlobalMappingNode node;
  ros::spin();

  return 0;
}