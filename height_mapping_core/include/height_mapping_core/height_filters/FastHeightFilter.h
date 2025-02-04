/*
 * FastHeightFilter.h
 *
 *  Created on: Dec 4, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#pragma once

#include "height_mapping_core/height_map/CloudTypes.h"
#include <pcl/point_cloud.h>

namespace height_mapping {

class FastHeightFilter {
public:
  using Ptr = std::shared_ptr<FastHeightFilter>;

  FastHeightFilter(double minZ, double maxZ);

  template <typename PointT>
  void filter(const typename pcl::PointCloud<PointT>::Ptr &cloud,
              typename pcl::PointCloud<PointT>::Ptr &cloudFiltered);

private:
  double minZ_, maxZ_;
};

} // namespace height_mapping
