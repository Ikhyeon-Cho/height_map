#include "height_mapping_data_collection/KITTIMapWriter.h"
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

void KITTIMapWriter::setDataPath(const std::string &path) {
  dataPath_ = path;
  std::filesystem::create_directories(dataPath_);
  isInitialized_ = true;
}

void KITTIMapWriter::write(const grid_map::GridMap &map,
                           const std::vector<std::string> &layers) {

  if (!isInitialized_) {
    throw std::runtime_error("[KITTIMapWriter] Data path is not set");
  }
  if (dataCount_ == 0) {
    writeMetadata(map, layers);
  }

  // Write each layer separately in row-major order
  for (const auto &layer : layers) {

    const auto &data = map.get(layer);
    auto filename = getMapFilename(dataCount_, layer);

    writeLayer(filename, data);
  }
  dataCount_++;
}

void KITTIMapWriter::writeMetadata(
    const grid_map::GridMap &map,
    const std::vector<std::string> &layers) const {

  auto path = dataPath_ + "/map_info.yaml";
  std::ofstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open map info: " + path);
  }

  // Write map geometry
  file << "size: [" << map.getSize()(0) << ", " << map.getSize()(1) << "]\n";
  file << "resolution: " << map.getResolution() << "\n";
  file << "position: [" << map.getPosition().x() << ", "
       << map.getPosition().y() << "]\n";

  // Write layer names
  file << "layers: [";
  for (size_t i = 0; i < layers.size(); ++i) {
    file << (i == 0 ? "" : ", ") << "\"" << layers[i] << "\"";
  }
  file << "]\n";
  file.close();
}

void KITTIMapWriter::writeLayer(const std::string &filename,
                                const Eigen::MatrixXf &data) const {

  std::ofstream file(filename, std::ios::binary);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open file: " + filename);
  }

  // Due to coordinate system, we need to flip the data
  // eval() is needed to make sure the data is flipped
  Eigen::MatrixXf dataFlipped =
      data.rowwise().reverse().eval().colwise().reverse().eval();
  Eigen::MatrixXf dataFlippedTransposed = dataFlipped.transpose();

  // Write data in column-major order to use in numpy later
  // Eigen uses row-major order by default
  file.write(reinterpret_cast<const char *>(dataFlippedTransposed.data()),
             dataFlippedTransposed.size() * sizeof(float));
  file.close();
}

std::string KITTIMapWriter::getMapFilename(unsigned int count,
                                           const std::string &layer) const {
  std::stringstream ss;
  ss << std::setw(6) << std::setfill('0') << count;
  return dataPath_ + "/" + ss.str() + "." + layer;
}
