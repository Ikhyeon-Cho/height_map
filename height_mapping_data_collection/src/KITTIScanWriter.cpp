#include "height_mapping_data_collection/KITTIScanWriter.h"
#include <filesystem>
#include <fstream>
#include <iomanip>

void KITTIScanWriter::setDataPath(const std::string &path) {

  dataPath_ = path;
  std::filesystem::create_directories(dataPath_);
  // Clear pose file if it exists
  auto poseFile = dataPath_ + "/poses.txt";
  std::ofstream(poseFile, std::ios::trunc).close();
  isInitialized_ = true;
}

bool KITTIScanWriter::writeScan(const CloudPtr &cloud, const Transform &pose) {
  // First write the scan, then write the pose
  if (!writeScan(cloud)) {
    return false;
  }
  return writePose(pose);
}

bool KITTIScanWriter::writeScan(const CloudPtr &cloud) {
  if (!isInitialized_) {
    throw std::runtime_error("[KITTIScanWriter] Data path is not set");
  }

  // Save point cloud in KITTI binary format
  try {
    auto scanFilename = getScanFilename(scanCount_);

    std::ofstream file(scanFilename, std::ios::out | std::ios::binary);
    if (!file.is_open()) {
      throw std::runtime_error("Failed to open file: " + scanFilename);
    }

    for (const auto &point : cloud->points) {
      file.write(reinterpret_cast<const char *>(&point.x), sizeof(float));
      file.write(reinterpret_cast<const char *>(&point.y), sizeof(float));
      file.write(reinterpret_cast<const char *>(&point.z), sizeof(float));
      file.write(reinterpret_cast<const char *>(&point.intensity),
                 sizeof(float));
    }
    file.close();

    scanCount_++;
    return true;

  } catch (const std::exception &e) {
    throw std::runtime_error("[KITTIScanWriter] Error writing scan: " +
                             std::string(e.what()));
    return false;
  }
}

std::string KITTIScanWriter::getScanFilename(unsigned int count) const {
  std::stringstream ss;
  ss << std::setw(6) << std::setfill('0') << count;
  return dataPath_ + "/" + ss.str() + ".bin";
}

bool KITTIScanWriter::writePose(const Transform &pose) {
  if (!isInitialized_) {
    throw std::runtime_error("[KITTIScanWriter] Data path is not set");
  }

  auto poseFile = dataPath_ + "/poses.txt";
  std::ofstream file(poseFile, std::ios::app);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open pose file: " + poseFile);
  }

  // Get rotation matrix and translation vector
  Eigen::Matrix3d rotMat = pose.rotation();
  Eigen::Vector3d trans = pose.translation();

  // Write transformation matrix in KITTI format (row-major order)
  // Save as: r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz
  // T = | r11 r12 r13 tx |
  //     | r21 r22 r23 ty |
  //     | r31 r32 r33 tz |
  //     |  0   0   0   1 |
  file << std::fixed << std::setprecision(6);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      file << rotMat(i, j) << " ";
    }
    file << trans[i] << " ";
  }
  file << std::endl;
  file.close();

  return true;
}
