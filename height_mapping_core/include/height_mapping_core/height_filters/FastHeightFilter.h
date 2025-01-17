/*
 * FastHeightFilter.h
 *
 *  Created on: Dec 4, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#pragma once

#include <pcl/point_cloud.h>

class FastHeightFilter {
public:
  using Ptr = std::shared_ptr<FastHeightFilter>;

  FastHeightFilter(float minZ, float maxZ);

  template <typename PointT>
  void filter(const typename pcl::PointCloud<PointT>::Ptr &cloud,
              typename pcl::PointCloud<PointT>::Ptr &cloudFiltered);

private:
  float minZ_, maxZ_;
};
