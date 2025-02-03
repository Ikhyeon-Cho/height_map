/*
 * global_mapping_node.cpp
 *
 *  Created on: Nov 24, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_ros/nodes/global_mapping_node.h"

GlobalMappingNode::GlobalMappingNode() {

  getNodeParameters();

  getFrameIDs();

  initializeTimers();

  setupROSInterface();

  globalMapper_ = std::make_unique<GlobalMapper>(getGlobalMappingParameters());
}

void GlobalMappingNode::getNodeParameters() {
  mapPublishRate_ = nhPriv_.param<double>("mapPublishRate", 10.0);
}

void GlobalMappingNode::getFrameIDs() {
  baselinkFrame_ = nhFrameID_.param<std::string>("base_link", "base_link");
  mapFrame_ = nhFrameID_.param<std::string>("map", "map");
  lidarFrame_ = nhFrameID_.param<std::string>("lidar", "velodyne");
}

void GlobalMappingNode::initializeTimers() {
  mapPublishTimer_ =
      nhPriv_.createTimer(ros::Duration(1.0 / mapPublishRate_),
                          &GlobalMappingNode::publishMap, this, false, false);
}

void GlobalMappingNode::setupROSInterface() {
  // Subscribers
  subLaserCloud_ = nh_.subscribe("/height_mapping/mapping/lasercloud", 1,
                                 &GlobalMappingNode::laserCloudCallback, this);
  subRGBCloud_ = nh_.subscribe("/height_mapping/mapping/rgbdcloud", 1,
                               &GlobalMappingNode::rgbCloudCallback, this);

  // Publishers
  pubGlobalMap_ = nh_.advertise<sensor_msgs::PointCloud2>(
      "/height_mapping/globalmap/pointcloud", 1);
  pubMapRegion_ = nh_.advertise<visualization_msgs::Marker>(
      "/height_mapping/globalmap/region", 1);

  // Services
  srvSaveMapToBag_ =
      nh_.advertiseService("/height_mapping/global_mapping/save_map",
                           &GlobalMappingNode::saveMapCallback, this);
  srvClearMap_ =
      nh_.advertiseService("/height_mapping/global_mapping/clear_map",
                           &GlobalMappingNode::clearMapCallback, this);
}

GlobalMapper::Config GlobalMappingNode::getGlobalMappingParameters() {

  GlobalMapper::Config cfg;
  cfg.frame_id = mapFrame_;
  cfg.estimator_type =
      nhGlobalMap_.param<std::string>("heightEstimatorType", "StatMean");
  cfg.grid_resolution = nhGlobalMap_.param<double>("gridResolution", 0.1);
  cfg.map_length_x = nhGlobalMap_.param<double>("mapLengthX", 400.0);
  cfg.map_length_y = nhGlobalMap_.param<double>("mapLengthY", 400.0);

  mapSavePath_ = nhGlobalMap_.param<std::string>(
      "mapSavePath",
      std::string("/home/") + std::getenv("USER") + "/Downloads");
  return cfg;
}

// Use the preprocessed cloud in height mapping node
void GlobalMappingNode::laserCloudCallback(
    const sensor_msgs::PointCloud2Ptr &msg) {

  if (!laserReceived_) {
    laserReceived_ = true;
    mapPublishTimer_.start();
    std::cout
        << "\033[1;32m[HeightMapping::GlobalMapping]: Laser cloud received! "
        << "Start global mapping... \033[0m\n";
  }
  pcl::PointCloud<Laser> cloud;
  pcl::moveFromROSMsg(*msg, cloud);
  globalMapper_->mapping(cloud);

  auto [get, laser2Map] = tf_.getTransform(lidarFrame_, mapFrame_);
  if (!get)
    return;
  Eigen::Vector3f laserPosition3D(laser2Map.transform.translation.x,
                                  laser2Map.transform.translation.y,
                                  laser2Map.transform.translation.z);
  globalMapper_->raycasting(laserPosition3D, cloud);
}

void GlobalMappingNode::rgbCloudCallback(
    const sensor_msgs::PointCloud2Ptr &msg) {

  if (!rgbReceived_) {
    rgbReceived_ = true;
    mapPublishTimer_.start();
    std::cout
        << "\033[1;32m[HeightMapping::GlobalMapping]: Colored cloud received! "
        << "Start global mapping... \033[0m\n";
  }
  pcl::PointCloud<Color> cloud;
  pcl::moveFromROSMsg(*msg, cloud);
  globalMapper_->mapping(cloud);
}

void GlobalMappingNode::publishMap(const ros::TimerEvent &) {

  std::vector<std::string> layers = {
      grid_map::HeightMap::CoreLayers::ELEVATION,
      grid_map::HeightMap::CoreLayers::ELEVATION_MAX,
      grid_map::HeightMap::CoreLayers::ELEVATION_MIN,
      grid_map::HeightMap::CoreLayers::VARIANCE,
      grid_map::HeightMap::CoreLayers::N_MEASUREMENTS,

  };
  // Visualize global map
  sensor_msgs::PointCloud2 msgCloud;
  toPointCloud2(globalMapper_->getHeightMap(), layers,
                globalMapper_->getMeasuredGridIndices(), msgCloud);
  pubGlobalMap_.publish(msgCloud);

  // Visualize map region
  visualization_msgs::Marker msgRegion;
  HeightMapMsgs::toMapRegion(globalMapper_->getHeightMap(), msgRegion);
  pubMapRegion_.publish(msgRegion);
}

void GlobalMappingNode::toPointCloud2(
    const grid_map::HeightMap &map, const std::vector<std::string> &layers,
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
  std::unordered_map<std::string, sensor_msgs::PointCloud2Iterator<float>>
      iterators;
  for (const auto &name : fieldNames) {
    iterators.emplace(name,
                      sensor_msgs::PointCloud2Iterator<float>(cloud, name));
  }

  // Fill point cloud data
  size_t validPoints = 0;
  for (const auto &index : measuredIndices) {
    grid_map::Position3 position;
    if (!map.getPosition3(grid_map::HeightMap::CoreLayers::ELEVATION, index,
                          position)) {
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

bool GlobalMappingNode::clearMapCallback(std_srvs::Empty::Request &req,
                                         std_srvs::Empty::Response &res) {
  globalMapper_->clearMap();
  return true;
}

bool GlobalMappingNode::saveMapCallback(std_srvs::Empty::Request &req,
                                        std_srvs::Empty::Response &res) {
  try {
    // Folder check and creation
    std::filesystem::path save_path(mapSavePath_);
    std::filesystem::path save_dir =
        save_path.has_extension() ? save_path.parent_path() : save_path;

    if (!std::filesystem::exists(save_dir)) {
      std::filesystem::create_directories(save_dir);
    }

    // Save GridMap to bag
    mapWriter_.writeToBag(globalMapper_->getHeightMap(), mapSavePath_,
                          "/height_mapping/globalmap/gridmap");

    // Save GridMap to PCD
    mapWriter_.writeToPCD(globalMapper_->getHeightMap(),
                          mapSavePath_.substr(0, mapSavePath_.rfind('.')) +
                              ".pcd");

    std::cout << "\033[1;33m[HeightMapping::GlobalMapping]: Successfully saved "
              << "map to " << mapSavePath_ << "\033[0m\n";

  } catch (const std::exception &e) {
    std::cout
        << "\033[1;31m[HeightMapping::GlobalMapping]: Failed to save map: "
        << std::string(e.what()) << "\033[0m\n";
  }

  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "global_mapping");
  GlobalMappingNode node;

  ros::spin();

  return 0;
}