#include "height_mapping_data_collection/DataCollectionNode.h"
#include <chrono>
#include <fstream>
#include <tf2/utils.h>

DataCollectionNode::DataCollectionNode() {

  getNodeParameters();
  getFrameIDs();
  setupROSInterface();

  getDataCollectionParameters();
  initialize();

  std::cout
      << "\033[1;32m[HeightMapping::DataCollection]: Data collection node "
         "initialized. Waiting for scan input... \033[0m\n";
}

void DataCollectionNode::getNodeParameters() {
  // Topics
  subLidarTopic_ =
      nhPriv_.param<std::string>("lidarCloudTopic", "/velodyne/points");

  // Paths
  nhPriv_.getParam("globalMapPath", globalMapPath_);
  nhPriv_.getParam("savePath", dataCollectionPath_);

  // Timers
  dataCollectionPeriod_ =
      nhPriv_.param<double>("dataCollectionPeriod", 1.0);    // [s]
  publishRate_ = nhPriv_.param<double>("publishRate", 10.0); // Hz
}

void DataCollectionNode::getFrameIDs() {
  mapFrame_ = nhFrameID_.param<std::string>("map", "map");
  baselinkFrame_ = nhFrameID_.param<std::string>("base_link", "base_link");
}

void DataCollectionNode::initializeTimers() {

  dataCollectionTimer_ = nh_.createTimer(
      ros::Duration(dataCollectionPeriod_),
      &DataCollectionNode::dataCollectionTimerCallback, this, false, false);
  publishTimer_ = nh_.createTimer(ros::Duration(1.0 / publishRate_),
                                  &DataCollectionNode::publishTimerCallback,
                                  this, false, false);
}

void DataCollectionNode::setupROSInterface() {
  // Subscribers
  subLidarScan_ = nh_.subscribe(subLidarTopic_, 1,
                                &DataCollectionNode::laserCloudCallback, this);
  // Publishers
  pubHeightMap_ = nh_.advertise<grid_map_msgs::GridMap>(
      "/height_mapping/data_collection/map", 1);
  pubScan_ = nh_.advertise<sensor_msgs::PointCloud2>(
      "/height_mapping/data_collection/scan", 1);

  // Service
  startCollectionServer_ = nhPriv_.advertiseService(
      "save_map", &DataCollectionNode::startMapSaverCallback, this);
}

void DataCollectionNode::getDataCollectionParameters() {

  // Height map parameters
  minHeightThreshold_ =
      nhDataCollection_.param<double>("minHeightThreshold", -0.2); // [m]
  maxHeightThreshold_ =
      nhDataCollection_.param<double>("maxHeightThreshold", 1.5);       // [m]
  mapLength_.x() = nhDataCollection_.param<double>("mapLengthX", 15.0); // [m]
  mapLength_.y() = nhDataCollection_.param<double>("mapLengthY", 15.0); // [m]
}

void DataCollectionNode::initialize() {

  globalMap_ = mapReader_.openBagFile(globalMapPath_,
                                      "/height_mapping/globalmap/gridmap");
  heightFilter_ = std::make_shared<height_mapping::FastHeightFilter>(
      minHeightThreshold_, maxHeightThreshold_);
  scanWriter_.setDataPath(dataCollectionPath_ + "/velodyne/");
  mapWriter_.setDataPath(dataCollectionPath_ + "/heightmap/");
}

void DataCollectionNode::laserCloudCallback(
    const sensor_msgs::PointCloud2Ptr &msg) {

  // Get Transform matrix: lidar -> baselink -> map
  const auto &lidarFrame = msg->header.frame_id;
  auto [get1, lidar2Baselink] = tf_.getTransform(lidarFrame, baselinkFrame_);
  auto [get2, baselink2Map] = tf_.getTransform(baselinkFrame_, mapFrame_);
  if (!get1 || !get2)
    return;

  // Prepare pointcloud
  auto inputCloud = boost::make_shared<pcl::PointCloud<Laser>>();
  processedCloud_ = boost::make_shared<pcl::PointCloud<Laser>>();

  // Get pointcloud in baselink frame
  pcl::moveFromROSMsg(*msg, *inputCloud);
  auto baselinkCloud =
      utils::pcl::transformPointcloud<Laser>(inputCloud, lidar2Baselink);

  // Apply height filter
  auto baselinkCloudFiltered = boost::make_shared<pcl::PointCloud<Laser>>();
  heightFilter_->filter<Laser>(baselinkCloud, baselinkCloudFiltered);

  // Filter pointcloud by x-y range
  auto range = mapLength_ / 2.0;
  processedCloud_ = utils::pcl::filterPointcloudByField<Laser>(
      baselinkCloudFiltered, "x", -range.x(), range.x());
  processedCloud_ = utils::pcl::filterPointcloudByField<Laser>(
      processedCloud_, "y", -range.y(), range.y());

  // transform pointcloud to map frame
  processedCloud_ =
      utils::pcl::transformPointcloud<Laser>(processedCloud_, baselink2Map);

  // Get robot-centric heightmap
  grid_map::Position robotPosition(baselink2Map.transform.translation.x,
                                   baselink2Map.transform.translation.y);
  if (!getSubMap(robotPosition, heightMap_))
    return;

  // height update
  updateCurrentHeightMap(heightMap_, processedCloud_);

  // Need this for robot-centric coordinates
  heightMap_.setPosition(heightMap_.getPosition() - robotPosition);
  heightMap_.getHeightMatrix().array() -= baselink2Map.transform.translation.z;
  for (auto &point : processedCloud_->points) {
    point.x -= baselink2Map.transform.translation.x;
    point.y -= baselink2Map.transform.translation.y;
    point.z -= baselink2Map.transform.translation.z;
  }

  // If not, start timer
  dataCollectionTimer_.start();
  publishTimer_.start();
}

bool DataCollectionNode::getSubMap(const grid_map::Position &position,
                                   grid_map::GridMap &map) {
  bool success{false};
  map = globalMap_.getSubmap(position, mapLength_, success);
  return success;
}

void DataCollectionNode::updateCurrentHeightMap(
    grid_map::HeightMap &map, const pcl::PointCloud<Laser>::ConstPtr &cloud) {

  grid_map::Index measuredIndex;
  grid_map::Position measuredPosition;

  auto &heightMatrix = map.getHeightMatrix();
  auto &minHeightMatrix = map.getMinHeightMatrix();
  auto &maxHeightMatrix = map.getMaxHeightMatrix();

  for (auto &point : cloud->points) {
    measuredPosition << point.x, point.y;
    if (!map.getIndex(measuredPosition, measuredIndex))
      continue;

    auto &height = heightMatrix(measuredIndex(0), measuredIndex(1));
    auto &minHeight = minHeightMatrix(measuredIndex(0), measuredIndex(1));
    auto &maxHeight = maxHeightMatrix(measuredIndex(0), measuredIndex(1));

    if (!map.isValid(measuredIndex)) {
      height = point.z;
      minHeight = point.z;
      maxHeight = point.z;
      continue;
    }

    auto alpha = 0.6;
    height = alpha * height + (1 - alpha) * point.z;
    minHeight = alpha * minHeight + (1 - alpha) * point.z;
    maxHeight = alpha * maxHeight + (1 - alpha) * point.z;
  }
}

void DataCollectionNode::dataCollectionTimerCallback(
    const ros::TimerEvent &event) {
  // Process and save like semantic KITTI format
  scanWriter_.writeScan(processedCloud_);

  // Save height map
  try {
    if (!mapWriter_.writeMap(heightMap_)) {
      std::cout << "\033[1;33m[DataCollectionNode]: Failed to write height "
                   "map\033[0m\n";
    }
  } catch (const std::exception &e) {
    std::cout << "\033[1;31m[DataCollectionNode]: Error writing height map: "
              << e.what() << "\033[0m\n";
  }
}

void DataCollectionNode::publishTimerCallback(const ros::TimerEvent &event) {

  // Publish pointcloud
  sensor_msgs::PointCloud2 cloudMsg;
  pcl::toROSMsg(*processedCloud_, cloudMsg);
  pubScan_.publish(cloudMsg);

  // Publish height map
  grid_map_msgs::GridMap heightmapMsg;
  grid_map::GridMapRosConverter::toMessage(heightMap_, heightmapMsg);
  pubHeightMap_.publish(heightmapMsg);
}

bool DataCollectionNode::startMapSaverCallback(std_srvs::Empty::Request &req,
                                               std_srvs::Empty::Response &res) {
  // TODO: Implement
  return true;
}
