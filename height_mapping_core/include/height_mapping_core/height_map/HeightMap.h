/*
 * HeightMap.h
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#pragma once
#include <grid_map_core/grid_map_core.hpp>

namespace grid_map {

class HeightMap : public GridMap {
public:
  // Layer names for core functionality
  struct CoreLayers {
    static constexpr const char *ELEVATION = "elevation";
    static constexpr const char *ELEVATION_MIN = "elevation_min";
    static constexpr const char *ELEVATION_MAX = "elevation_max";
    static constexpr const char *VARIANCE = "variance";
    static constexpr const char *N_MEASUREMENTS = "n_measured";
  };

  HeightMap();

  // Layer management
  void addLayer(const std::string &layer, float default_value = NAN);
  void addBasicLayer(const std::string &layer);
  bool hasLayer(const std::string &layer) const { return exists(layer); }
  void removeLayer(const std::string &layer);

  // Core layer accessors
  Matrix &getHeightMatrix() { return get(CoreLayers::ELEVATION); }
  Matrix &getMinHeightMatrix() { return get(CoreLayers::ELEVATION_MIN); }
  Matrix &getMaxHeightMatrix() { return get(CoreLayers::ELEVATION_MAX); }
  Matrix &getVarianceMatrix() { return get(CoreLayers::VARIANCE); }
  Matrix &getMeasurementCountMatrix() {
    return get(CoreLayers::N_MEASUREMENTS);
  }
  const Matrix &getHeightMatrix() const { return get(CoreLayers::ELEVATION); }
  const Matrix &getVarianceMatrix() const { return get(CoreLayers::VARIANCE); }
  const Matrix &getMinHeightMatrix() const {
    return get(CoreLayers::ELEVATION_MIN);
  }
  const Matrix &getMaxHeightMatrix() const {
    return get(CoreLayers::ELEVATION_MAX);
  }
  const Matrix &getMeasurementCountMatrix() const {
    return get(CoreLayers::N_MEASUREMENTS);
  }

  // Cell validity checks
  bool isEmptyAt(const Index &index) const { return !isValid(index); }
  bool isEmptyAt(const std::string &layer, const Index &index) const {
    return !exists(layer) || !std::isfinite(at(layer, index));
  }

  // Height statistics
  float getMinHeight() const;
  float getMaxHeight() const;
  bool hasHeightValues() const;

  bool is_initialized_{false};
};

class HeightMapMath {
  // min max operation reference:
  // https://www.geeksforgeeks.org/difference-between-stdnumeric_limitst-min-max-and-lowest-in-cpp/

public:
  static float getMinVal(const grid_map::HeightMap &map,
                         const std::string &layer) {
    const auto &data = map[layer];

    auto fillNaNForFindingMinVal =
        data.array().isNaN().select(std::numeric_limits<double>::max(), data);
    return fillNaNForFindingMinVal.minCoeff();
  }

  static float getMaxVal(const grid_map::HeightMap &map,
                         const std::string &layer) {
    const auto &data = map[layer];

    auto fillNaNForFindingMaxVal = data.array().isNaN().select(
        std::numeric_limits<double>::lowest(), data);
    return fillNaNForFindingMaxVal.maxCoeff();
  }
};

} // namespace grid_map
