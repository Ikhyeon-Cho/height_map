/*
 * sensor_processor_node.cpp
 *
 *  Created on: Nov 25, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_ros/nodes/sensor_processor_node.h"

SensorProcessorNode::SensorProcessorNode() {

  getNodeParameters();
  getFrameIDs();
  setupROSInterface();

  getProcessingParameters();
  std::cout << "\033[1;32m[HeightMapping::SensorProcessor]: Sensor processor "
               "initialized. Waiting for "
            << inputTopics_.size() << " rgb clouds... \033[0m\n";
}

void SensorProcessorNode::getNodeParameters() {
  // Get topic names
  if (nhPriv_.hasParam("rgbCloudTopics")) {
    nhPriv_.getParam("rgbCloudTopics", inputTopics_);
  } else {
    inputTopics_ = {"/sensor1/points", "/sensor2/points", "/sensor3/points"};
  }
  outputCloudTopic_ =
      nhPriv_.param<std::string>("outputCloudTopic", "points");
}

void SensorProcessorNode::getFrameIDs() {
  baselinkFrame_ = nhFrameID_.param<std::string>("base_link", "base_link");
}

void SensorProcessorNode::setupROSInterface() {
  // Subscribers
  size_t queue_size = 10;
  for (const auto &topic : inputTopics_) {
    cloudSubs_.push_back(
        std::make_shared<CloudSubscriber>(nh_, topic, queue_size));
  }
  // Synchronizer: Based on number of inputs (2 or 3)
  if (cloudSubs_.size() == 2) {
    sync2_.reset(new Synchronizer2(SyncPolicy2(queue_size), *cloudSubs_[0],
                                   *cloudSubs_[1]));
    sync2_->registerCallback(
        boost::bind(&SensorProcessorNode::syncCallback2, this, _1, _2));
  } else if (cloudSubs_.size() == 3) {
    sync3_.reset(new Synchronizer3(SyncPolicy3(queue_size), *cloudSubs_[0],
                                   *cloudSubs_[1], *cloudSubs_[2]));
    sync3_->registerCallback(
        boost::bind(&SensorProcessorNode::syncCallback3, this, _1, _2, _3));
  }
  // Publisher
  pubProcessedCloud_ =
      nh_.advertise<sensor_msgs::PointCloud2>(outputCloudTopic_, 1);
}

void SensorProcessorNode::getProcessingParameters() {
  downsamplingResolution_ =
      nhSensorProcessor_.param<double>("downsamplingResolution", 0.1);
  minRangeThreshold_ =
      nhSensorProcessor_.param<double>("minRangeThreshold", 0.3);
  maxRangeThreshold_ =
      nhSensorProcessor_.param<double>("maxRangeThreshold", 5.0);
}

void SensorProcessorNode::syncCallback2(
    const sensor_msgs::PointCloud2ConstPtr &msg1,
    const sensor_msgs::PointCloud2ConstPtr &msg2) {

  // Convert to PCL (be careful with const_cast)
  pcl::moveFromROSMsg(*const_cast<sensor_msgs::PointCloud2 *>(msg1.get()),
                      *cloud1_);
  pcl::moveFromROSMsg(*const_cast<sensor_msgs::PointCloud2 *>(msg2.get()),
                      *cloud2_);

  // Downsampling
  downsampled1_ = utils::pcl::filterPointcloudByVoxel<Color>(
      cloud1_, downsamplingResolution_);
  downsampled2_ = utils::pcl::filterPointcloudByVoxel<Color>(
      cloud2_, downsamplingResolution_);

  // Free memory
  cloud1_->clear();
  cloud2_->clear();

  // Transform each cloud to baselink frame
  transformToBaselink(downsampled1_, transformed1_, msg1->header.frame_id);
  transformToBaselink(downsampled2_, transformed2_, msg2->header.frame_id);

  // Free memory
  downsampled1_->clear();
  downsampled2_->clear();

  // Filter each cloud
  filtered1_ = utils::pcl::filterPointcloudByRange<Color>(
      transformed1_, minRangeThreshold_, maxRangeThreshold_);
  filtered2_ = utils::pcl::filterPointcloudByRange<Color>(
      transformed2_, minRangeThreshold_, maxRangeThreshold_);

  // Free memory
  transformed1_->clear();
  transformed2_->clear();

  // Merge transformed clouds
  *filtered1_ += *filtered2_;

  // Convert back to ROS message
  sensor_msgs::PointCloud2 outMsg;
  pcl::toROSMsg(*filtered1_, outMsg);
  outMsg.header.frame_id = baselinkFrame_;
  outMsg.header.stamp = msg1->header.stamp; // Use timestamp from first message
  pubProcessedCloud_.publish(outMsg);

  // Free memory
  filtered1_->clear();
  filtered2_->clear();
}

void SensorProcessorNode::syncCallback3(
    const sensor_msgs::PointCloud2ConstPtr &msg1,
    const sensor_msgs::PointCloud2ConstPtr &msg2,
    const sensor_msgs::PointCloud2ConstPtr &msg3) {

  // Convert to PCL (be careful with const_cast)
  pcl::moveFromROSMsg(*const_cast<sensor_msgs::PointCloud2 *>(msg1.get()),
                      *cloud1_);
  pcl::moveFromROSMsg(*const_cast<sensor_msgs::PointCloud2 *>(msg2.get()),
                      *cloud2_);
  pcl::moveFromROSMsg(*const_cast<sensor_msgs::PointCloud2 *>(msg3.get()),
                      *cloud3_);

  // Downsampling
  downsampled1_ = utils::pcl::filterPointcloudByVoxel<Color>(
      cloud1_, downsamplingResolution_);
  downsampled2_ = utils::pcl::filterPointcloudByVoxel<Color>(
      cloud2_, downsamplingResolution_);
  downsampled3_ = utils::pcl::filterPointcloudByVoxel<Color>(
      cloud3_, downsamplingResolution_);

  // Free memory
  cloud1_->clear();
  cloud2_->clear();
  cloud3_->clear();

  // Transform each cloud to baselink frame
  transformToBaselink(downsampled1_, transformed1_, msg1->header.frame_id);
  transformToBaselink(downsampled2_, transformed2_, msg2->header.frame_id);
  transformToBaselink(downsampled3_, transformed3_, msg3->header.frame_id);

  // Free memory
  downsampled1_->clear();
  downsampled2_->clear();
  downsampled3_->clear();

  // Filter each cloud
  filtered1_ = utils::pcl::filterPointcloudByRange<Color>(
      transformed1_, minRangeThreshold_, maxRangeThreshold_);
  filtered2_ = utils::pcl::filterPointcloudByRange<Color>(
      transformed2_, minRangeThreshold_, maxRangeThreshold_);
  filtered3_ = utils::pcl::filterPointcloudByRange<Color>(
      transformed3_, minRangeThreshold_, maxRangeThreshold_);

  // Free memory
  transformed1_->clear();
  transformed2_->clear();
  transformed3_->clear();

  // Merge transformed clouds
  *filtered1_ += *filtered2_;
  *filtered1_ += *filtered3_;

  // Convert back to ROS message
  sensor_msgs::PointCloud2 outMsg;
  pcl::toROSMsg(*filtered1_, outMsg);
  outMsg.header.frame_id = baselinkFrame_;
  outMsg.header.stamp = msg1->header.stamp;
  pubProcessedCloud_.publish(outMsg);

  // Free memory
  filtered1_->clear();
  filtered2_->clear();
  filtered3_->clear();
}

void SensorProcessorNode::transformToBaselink(
    const pcl::PointCloud<Color>::Ptr &cloud,
    pcl::PointCloud<Color>::Ptr &transformedCloud,
    const std::string &sensorFrame) {

  // Get transform matrix
  auto [success, transform] = tf_.getTransform(
      sensorFrame, baselinkFrame_, ros::Time(0), ros::Duration(0.1));

  // Transform point cloud
  transformedCloud = utils::pcl::transformPointcloud<Color>(cloud, transform);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "sensor_processor_node");
  SensorProcessorNode node;

  ros::spin();

  return 0;
}