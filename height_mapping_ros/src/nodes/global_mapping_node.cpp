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
#include <height_mapping_msgs/HeightMapMsgs.h>

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

  // Use the preprocessed cloud in height mapping node
  // Subscribers
  sub_lidarscan_ = nh_.subscribe(cfg_.lidarcloud_topic, 1, &GlobalMappingNode::lidarScanCallback, this);
  sub_rgbdscan_ = nh_.subscribe(cfg_.rgbdcloud_topic, 1, &GlobalMappingNode::rgbdScanCallback, this);

  // Publishers
  pub_map_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/height_mapping/global/map_cloud", 1);
  pub_map_region_ = nh_.advertise<visualization_msgs::Marker>("/height_mapping/global/map_region", 1);
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

  pcl::PointCloud<Laser> pointcloud;
  pcl::moveFromROSMsg(*msg, pointcloud);
  mapper_->mapping(pointcloud);

  // TODO: Raycasting needs sensor origin
  // geometry_msgs::TransformStamped t;
  // if (!tf_.lookupTransform(frameID.map, frameID.sensor, t))
  //   return;

  // Eigen::Vector3f laserPosition3D(t.transform.translation.x, t.transform.translation.y,
  //                                 t.transform.translation.z);
  // mapper_->raycasting(laserPosition3D, pointcloud);
}

void GlobalMappingNode::rgbdScanCallback(const sensor_msgs::PointCloud2Ptr &msg) {

  if (!rgbdscan_received_) {
    rgbdscan_received_ = true;
    frameID.sensor = msg->header.frame_id;
    map_publish_timer_.start();
    std::cout << "\033[1;32m[height_mapping_ros::GlobalMappingNode]: Colored cloud received! "
              << "Start global mapping... \033[0m\n";
  }

  pcl::PointCloud<Color> cloud;
  pcl::moveFromROSMsg(*msg, cloud);
  mapper_->mapping(cloud);
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
  HeightMapMsgs::toMapRegion(mapper_->getHeightMap(), msg_map_region);
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

bool GlobalMappingNode::clearMapCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {

  mapper_->clearMap();
  return true;
}

bool GlobalMappingNode::saveMapCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  try {
    // Folder check and creation
    std::filesystem::path save_path(mapSavePath_);
    std::filesystem::path save_dir = save_path.has_extension() ? save_path.parent_path() : save_path;

    if (!std::filesystem::exists(save_dir)) {
      std::filesystem::create_directories(save_dir);
    }

    // Save GridMap to bag
    mapWriter_.writeToBag(mapper_->getHeightMap(), mapSavePath_, "/height_mapping/globalmap/gridmap");

    // Save GridMap to PCD
    mapWriter_.writeToPCD(mapper_->getHeightMap(), mapSavePath_.substr(0, mapSavePath_.rfind('.')) + ".pcd");

    std::cout << "\033[1;33m[height_mapping_ros::GlobalMappingNode]: Successfully saved "
              << "map to " << mapSavePath_ << "\033[0m\n";

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