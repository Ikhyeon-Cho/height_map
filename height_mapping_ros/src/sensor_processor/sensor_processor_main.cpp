/*
 * sensor_processor_main.cpp
 *
 *  Created on: Nov 25, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_ros/SensorProcessorNode.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "sensor_processor_node");
  SensorProcessorNode node;

  ros::spin();

  return 0;
}