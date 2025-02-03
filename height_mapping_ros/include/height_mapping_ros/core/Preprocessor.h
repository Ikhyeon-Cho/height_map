#pragma once

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Core>
#include "height_mapping_ros/utils/pc_utils.h"

namespace height_mapping_ros {

template <typename PointT> struct preproc_result {
  typename pcl::PointCloud<PointT>::Ptr cloud_base;
  typename pcl::PointCloud<PointT>::Ptr cloud_filtered;
  typename pcl::PointCloud<PointT>::Ptr cloud_processed;
};

template <typename PointT>
preproc_result<PointT> preprocessPointCloud(const typename pcl::PointCloud<PointT>::Ptr &cloud_raw,
                                         const geometry_msgs::TransformStamped &sensor2base,
                                         const geometry_msgs::TransformStamped &base2map,
                                         const Eigen::Vector2f &filter_range) {
  preproc_result<PointT> result;
  result.sensor_cloud = raw_cloud;
  result.base_link_cloud = pc_utils::applyTransform(raw_cloud, sensor_to_base);
  auto cloud_filtered =
      pc_utils::passThrough(result.base_link_cloud, "x", -filter_range.x(), filter_range.x());
  cloud_filtered = pc_utils::passThrough(cloud_filtered, "y", -filter_range.y(), filter_range.y());
  result.filtered_cloud = cloud_filtered;
  result.map_cloud = pc_utils::applyTransform(cloud_filtered, base_to_map);
  return result;
}

} // namespace height_mapping_ros