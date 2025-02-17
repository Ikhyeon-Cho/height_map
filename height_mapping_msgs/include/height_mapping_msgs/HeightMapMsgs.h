/*
 *  HeightMapMsgs.h
 *  Created on: Dec 2, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAP_MSGS_H
#define HEIGHT_MAP_MSGS_H

#include <height_mapping_core/height_mapping_core.h>

#include "height_mapping_msgs/SaveLayerToImage.h"
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>

#include <grid_map_cv/GridMapCvConverter.hpp>

class HeightMapMsgs {
public:
  static void toMapRegion(const grid_map::HeightMap &map,
                          visualization_msgs::Marker &msg);

  static void toOccupancyGrid(const grid_map::HeightMap &map,
                              nav_msgs::OccupancyGrid &msg);
};

#endif // HEIGHT_MAP_MSGS_H