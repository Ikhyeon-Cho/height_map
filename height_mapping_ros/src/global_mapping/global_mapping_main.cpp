/*
 * global_mapping_main.cpp
 *
 *  Created on: Mar 20, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_ros/GlobalMappingNode.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "global_mapping");
  GlobalMappingNode node;

  ros::spin();

  return 0;
}