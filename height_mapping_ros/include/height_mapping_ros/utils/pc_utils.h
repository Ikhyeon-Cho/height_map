#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>

namespace pc_utils {

// Aliases for brevity.
template <typename T> using Cloud = pcl::PointCloud<T>;
template <typename T> using CloudPtr = typename Cloud<T>::Ptr;

/**
 * @brief Converts a geometry_msgs::Transform into an Eigen::Affine3d.
 *
 * @param t The geometry_msgs::Transform to convert.
 * @return An Eigen::Affine3d representation of the transform.
 */
inline Eigen::Affine3d toAffine3d(const geometry_msgs::Transform &t) {
  Eigen::Translation3d translation(t.translation.x, t.translation.y, t.translation.z);
  Eigen::Quaterniond rotation(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z);
  return translation * rotation;
}

/**
 * @brief Applies a transformation to the point cloud.
 *
 * If the input cloud is empty or its frame is not set, it is returned unmodified.
 *
 * @tparam T PCL point type.
 * @param input The input point cloud.
 * @param ts The transform to apply.
 * @return A new point cloud in the target frame.
 */
template <typename T>
inline CloudPtr<T> applyTransform(const CloudPtr<T> &input,
                                  const geometry_msgs::TransformStamped &ts) {
  if (input->header.frame_id.empty() || input->empty())
    return input;
  // If source and target frames match, no action is needed.
  if (ts.child_frame_id == ts.header.frame_id)
    return input;

  auto affine = toAffine3d(ts.transform);
  auto output = boost::make_shared<Cloud<T>>();
  pcl::transformPointCloud(*input, *output, affine);
  output->header = input->header;
  output->header.frame_id = ts.header.frame_id;
  return output;
}

/**
 * @brief Filters the point cloud using a pass-through filter on a specific field.
 *
 * @tparam T PCL point type.
 * @param input The input point cloud.
 * @param field Name of the field (e.g., "x", "y", "z").
 * @param minVal Minimum accepted value.
 * @param maxVal Maximum accepted value.
 * @param invert If true, keep points outside the range.
 * @return A new, filtered point cloud.
 */
template <typename T>
inline CloudPtr<T> passThrough(const CloudPtr<T> &input, const std::string &field, double minVal,
                               double maxVal, bool invert = false) {
  if (input->empty())
    return input;
  auto output = boost::make_shared<Cloud<T>>();
  pcl::PassThrough<T> pass;
  pass.setInputCloud(input);
  pass.setFilterFieldName(field);
  pass.setFilterLimits(minVal, maxVal);
  pass.setFilterLimitsNegative(invert);
  pass.filter(*output);
  output->header = input->header;
  return output;
}

/**
 * @brief Filters the point cloud on the XY plane based on range.
 *
 * @tparam T PCL point type.
 * @param input The input point cloud.
 * @param minRange Minimum radial distance in the XY plane.
 * @param maxRange Maximum radial distance in the XY plane.
 * @return A new point cloud with points in the specified 2D range.
 */
template <typename T>
inline CloudPtr<T> filter2D(const CloudPtr<T> &input, double minRange, double maxRange) {
  if (input->empty())
    return input;
  auto output = boost::make_shared<Cloud<T>>();
  output->header = input->header;
  for (const auto &pt : input->points) {
    double r = std::sqrt(pt.x * pt.x + pt.y * pt.y);
    if (r > minRange && r < maxRange)
      output->points.push_back(pt);
  }
  return output;
}

/**
 * @brief Filters the point cloud based on 3D range.
 *
 * @tparam T PCL point type.
 * @param input The input point cloud.
 * @param minRange Minimum 3D range.
 * @param maxRange Maximum 3D range.
 * @return A new point cloud containing points within the specified range.
 */
template <typename T>
inline CloudPtr<T> filter3D(const CloudPtr<T> &input, double minRange, double maxRange) {
  if (input->empty())
    return input;
  auto output = boost::make_shared<Cloud<T>>();
  output->header = input->header;
  for (const auto &pt : input->points) {
    double r = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
    if (r > minRange && r < maxRange)
      output->points.push_back(pt);
  }
  return output;
}

/**
 * @brief Filters the point cloud based on an angular range in the XY plane.
 *
 * Angles are specified in degrees. If invert is true, points outside the range are kept.
 *
 * @tparam T PCL point type.
 * @param input The input point cloud.
 * @param startDeg Starting angle in degrees.
 * @param endDeg Ending angle in degrees.
 * @param invert If true, inverts the filter.
 * @return A new point cloud with the angular filtering applied.
 */
template <typename T>
inline CloudPtr<T> filterAngle(const CloudPtr<T> &input, double startDeg, double endDeg,
                               bool invert = false) {
  if (input->empty())
    return input;

  auto norm = [](double a) -> double {
    while (a > 180.0)
      a -= 360.0;
    while (a < -180.0)
      a += 360.0;
    return a;
  };
  const double sRad = norm(startDeg) * M_PI / 180.0;
  const double eRad = norm(endDeg) * M_PI / 180.0;

  auto output = boost::make_shared<Cloud<T>>();
  output->header = input->header;
  for (const auto &pt : input->points) {
    double a = std::atan2(pt.y, pt.x);
    bool keep = (sRad <= eRad) ? (a >= sRad && a <= eRad) : (a >= sRad || a <= eRad);
    if (invert)
      keep = !keep;
    if (keep)
      output->points.push_back(pt);
  }
  return output;
}

/**
 * @brief Downsamples the point cloud via a voxel grid filter.
 *
 * @tparam T PCL point type.
 * @param input The input point cloud.
 * @param dx Leaf size along x.
 * @param dy Leaf size along y.
 * @param dz Leaf size along z.
 * @return A new, downsampled point cloud.
 */
template <typename T>
inline CloudPtr<T> downsampleVoxel(const CloudPtr<T> &input, double dx, double dy, double dz) {
  if (input->empty())
    return input;
  auto output = boost::make_shared<Cloud<T>>();
  pcl::VoxelGrid<T> vg;
  vg.setInputCloud(input);
  vg.setLeafSize(dx, dy, dz);
  vg.filter(*output);
  output->header = input->header;
  return output;
}

/**
 * @brief Isotropic voxel-grid downsampling.
 *
 * @tparam T PCL point type.
 * @param input The input point cloud.
 * @param leafSize The voxel size for all dimensions.
 * @return A new, downsampled point cloud.
 */
template <typename T>
inline CloudPtr<T> downsampleVoxel(const CloudPtr<T> &input, double leafSize) {
  return downsampleVoxel<T>(input, leafSize, leafSize, leafSize);
}

} // namespace pc_utils