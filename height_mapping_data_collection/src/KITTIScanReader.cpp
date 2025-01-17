#include "height_mapping_data_collection/KITTIScanReader.h"
#include <fstream>
#include <iomanip>
#include <sstream>

void KITTIScanReader::setDataPath(const std::string &path) {
  dataPath_ = path;
  isInitialized_ = true;
}

KITTIScanReader::CloudPtr
KITTIScanReader::readScan(unsigned int scanIndex) const {
  if (!isInitialized_) {
    throw std::runtime_error("[KITTIScanReader] Data path is not set");
  }

  auto cloud = boost::make_shared<Cloud>();
  auto filename = getScanFilename(scanIndex);

  std::ifstream file(filename, std::ios::in | std::ios::binary);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open file: " + filename);
  }

  // Read points until end of file
  float data[4]; // x,y,z,intensity
  while (file.read(reinterpret_cast<char *>(data), 4 * sizeof(float))) {
    PointXYZI point;
    point.x = data[0];
    point.y = data[1];
    point.z = data[2];
    point.intensity = data[3];
    cloud->points.emplace_back(point);
  }

  // Set cloud parameters
  cloud->width = cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = true;

  file.close();
  return cloud;
}

KITTIScanReader::TransformVector KITTIScanReader::readPoses() const {
  if (!isInitialized_) {
    throw std::runtime_error("[KITTIScanReader] Data path is not set");
  }

  // Read pose file
  auto poseFile = dataPath_ + "/poses.txt";
  std::ifstream file(poseFile);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open pose file: " + poseFile);
  }

  // Get poses in transforms vector
  TransformVector transforms;
  std::string line;
  while (std::getline(file, line)) {
    std::istringstream ss(line);
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();

    // Read 12 values (3x4 matrix)
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 4; ++j) {
        ss >> matrix(i, j);
      }
    }

    Transform transform(matrix);
    transforms.push_back(transform);
  }
  file.close();

  return transforms;
}

std::string KITTIScanReader::getScanFilename(unsigned int count) const {
  std::stringstream ss;
  ss << std::setw(6) << std::setfill('0') << count;
  return dataPath_ + "/" + ss.str() + ".bin";
}
