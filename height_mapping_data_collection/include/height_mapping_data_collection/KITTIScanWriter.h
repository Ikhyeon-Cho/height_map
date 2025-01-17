/*
 * KITTIScanWriter.h
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

class KITTIScanWriter {
public:
  using PointXYZI = pcl::PointXYZI;
  using Cloud = pcl::PointCloud<PointXYZI>;
  using CloudPtr = Cloud::Ptr;
  using Transform = Eigen::Affine3d;

  KITTIScanWriter() = default;

  void setDataPath(const std::string &path);
  bool writeScan(const CloudPtr &cloud);
  bool writeScan(const CloudPtr &cloud, const Transform &pose);

private:
  bool writePose(const Transform &pose);
  std::string getScanFilename(unsigned int count) const;

  std::string dataPath_;
  unsigned int scanCount_{0};
  bool isInitialized_{false};
};
