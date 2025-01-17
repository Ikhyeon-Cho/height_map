/*
 * height_mapping_main.cpp
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_ros/HeightMappingNode.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "height_mapping_node"); // launch file overrides this
  HeightMappingNode node;

  ros::spin();

  return 0;
}