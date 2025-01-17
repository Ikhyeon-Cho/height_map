#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from semantic_kitti_ros.point_msg import PointMsgXYZIRGB


class HeightMapVisualizerROS:
    def __init__(self, topic_name, frame_id="map"):
        self.pub = rospy.Publisher(topic_name, PointCloud2, queue_size=10)
        self.msg = PointMsgXYZIRGB(frame_id)

    def publish(self, heightmap_data, resolution, position):
        points = self._heightmap_to_points(
            heightmap_data, resolution, position)
        self.msg.create(points)
        self.pub.publish(self.msg.msg)

    def _heightmap_to_points(self, heightmap, resolution, position):
        H, W = heightmap.shape

        # Get valid indices (non-NaN values)
        valid_indices = np.stack(np.nonzero(
            ~np.isnan(heightmap)), axis=1)  # Shape (N, 2)

        # Extract height values at valid indices
        heights = heightmap[valid_indices[:, 0],
                            valid_indices[:, 1]]  # Shape (N,)

        # Convert indices to (x, y) coordinates
        # resolution + np.array(offset[:2])
        xy_coords = (valid_indices + 0.5) * 0.1 + np.array([-H/2*0.1, -W/2*0.1])  # Shape (N, 2)

        # Combine (x, y) and heights to form 3D points
        points = np.column_stack([xy_coords, heights])  # Shape (N, 3)

        return points
