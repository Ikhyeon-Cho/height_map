/*
 * KITTIScanReader.h
 *
 *  Created on: Dec 02, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <vector>

class KITTIScanReader {
public:
  using PointXYZI = pcl::PointXYZI;
  using Cloud = pcl::PointCloud<PointXYZI>;
  using CloudPtr = Cloud::Ptr;
  using Transform = Eigen::Affine3d;
  using TransformVector = std::vector<Transform>;

  KITTIScanReader() = default;

  void setDataPath(const std::string &path);
  CloudPtr readScan(unsigned int scanIndex) const;
  TransformVector readPoses() const;

private:
  std::string getScanFilename(unsigned int count) const;

  std::string dataPath_;
  bool isInitialized_{false};
};
