#!/usr/bin/env python3

import rospy
from heightmap import HeightMapReader
from visualizer import HeightMapVisualizerROS
import numpy as np


def main():
    rospy.init_node('heightmap_publisher', anonymous=True)
    heightmap_reader = HeightMapReader()
    visualizer = HeightMapVisualizerROS(topic_name='/heightmap_cloud')

    base_path = f'/home/ikhyeon/ros/dev_ws/src/height_mapping/data/heightmap/'
    frames = range(0, 10, 1)
    rate = rospy.Rate(2)  # 2 Hz

    while not rospy.is_shutdown():
        for frame in frames:
            start_time = rospy.get_rostime()

            # Construct file path
            frame_str = f"{0:06d}"
            file_path = f"{base_path}/{frame_str}"

            # Read and publish height map
            elevation = heightmap_reader.open_layer('elevation', file_path)
            visualizer.publish(elevation,
                               heightmap_reader.resolution,
                               heightmap_reader.position)

            elapsed = (rospy.get_rostime() - start_time).to_sec()
            print(f"Frame {frame_str} processed in {elapsed:.3f}s")

            rate.sleep()

            if rospy.is_shutdown():
                break

        if not rospy.is_shutdown():
            print("Restarting sequence...")


if __name__ == '__main__':
    main()
