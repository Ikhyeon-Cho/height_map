/*
 * KITTIMapWriter.h
 *
 *  Created on: Dec 02, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#pragma once

#include <height_mapping_core/height_mapping_core.h>
#include <string>
#include <vector>

class KITTIMapWriter {
public:
  using Ptr = std::unique_ptr<KITTIMapWriter>;

  KITTIMapWriter() = default;
  void setDataPath(const std::string &path);
  void write(const grid_map::GridMap &map,
             const std::vector<std::string> &layers);

private:
  void writeMetadata(const grid_map::GridMap &map,
                     const std::vector<std::string> &layers) const;
  std::string getMapFilename(unsigned int count,
                             const std::string &layer) const;

  void writeLayer(const std::string &filename,
                  const Eigen::MatrixXf &data) const;

  std::string dataPath_;
  bool isInitialized_{false};
  unsigned int dataCount_{0};
};
