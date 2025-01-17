"""
Author: Ikhyeon Cho
File: laserscan.py
Date: 2024/11/06 13:20

The custom I/O class for scan files using official semantic kitti api.
"""

import numpy as np
from semantic_kitti_api.api.laserscan import SemLaserScan
from semantic_kitti_api.utils.label import KITTILabels


class KITTIScanReader(SemLaserScan):
    def __init__(self):
        self._labeler = KITTILabels()
        super().__init__(
            sem_color_dict=self._labeler.color_map,
            project=False,  # We don't need projection
            H=64,          # Fixed for KITTI
            W=1024,        # Fixed for KITTI
            fov_up=3.0,    # Fixed for KITTI
            fov_down=-25.0  # Fixed for KITTI
        )

    def open_scan(self, path):
        """Read point cloud from .bin file"""
        super().open_scan(path)
        # Return pointcloud with intensity
        return np.column_stack((self.points, self.remissions))

    def get_points(self):
        if not self._has_points():
            raise RuntimeError("get_scan(): No points available")
        if self._has_intensity():
            return np.column_stack((self.points, self.remissions))
        else:
            return self.points

    def open_label(self, path, return_instance=False):
        """Read semantic labels from .label file"""
        super().open_label(path)
        if return_instance:
            return self.sem_label, self.inst_label
        else:
            return self.sem_label

    def get_labels(self, return_instance=False):
        """Get semantic and instance labels"""
        if not self._has_label():
            raise RuntimeError("get_label(): No labels available")
        if return_instance:
            if self.inst_label is None:
                raise ValueError(
                    "get_label(): Instance labels are not available")
            return self.sem_label, self.inst_label
        else:
            return self.sem_label

    def _has_points(self):
        return len(self.points) > 0

    def _has_intensity(self):
        return len(self.remissions) > 0

    def _has_label(self):
        return len(self.sem_label) > 0


class KITTIScanWriter:
    @staticmethod
    def write_scan(points, path):
        """Write point cloud to .bin file in KITTI format"""
        if not isinstance(points, np.ndarray):
            raise TypeError("write_scan(): Points should be numpy array")

        if points.shape[1] not in [3, 4]:
            raise ValueError(
                "write_scan(): Points should have shape (N, 3) or (N, 4)")

        # Ensure float32 format
        points = points.astype(np.float32)

        # If only xyz is provided, add zero intensity
        if points.shape[1] == 3:
            points = np.column_stack(
                (points, np.zeros(len(points), dtype=np.float32)))

        # Save to binary file
        points.tofile(path)

    @staticmethod
    def write_label(semantic_labels, path, instance_labels=None):
        """Write semantic and instance labels to .label file in KITTI format"""
        if not isinstance(semantic_labels, np.ndarray):
            raise TypeError("write_label(): Labels should be numpy array")

        # Convert to uint32
        semantic_labels = semantic_labels.astype(np.uint32)

        if instance_labels is None:
            instance_labels = np.zeros_like(semantic_labels, dtype=np.uint32)
        else:
            instance_labels = instance_labels.astype(np.uint32)

        if len(semantic_labels) != len(instance_labels):
            raise ValueError(
                "write_label(): Semantic and instance labels must have same length")

        # Combine semantic and instance labels
        # Upper half for instance, lower half for semantic
        labels = semantic_labels + (instance_labels << 16)

        # Save to binary file
        labels.tofile(path)


if __name__ == '__main__':

    # Example usage
    scan_reader = KITTIScanReader()
    scan_writer = KITTIScanWriter()

    scan_path = '/data/semanticKITTI/dataset/sequences/00/velodyne/000015.bin'
    label_path = '/data/semanticKITTI/dataset/sequences/00/labels/000015.label'

    # 1. open_scan() -> returns pointcloud with intensity
    points = scan_reader.open_scan(scan_path)
    print(f'points.shape: {points.shape}')  # (N x 4)

    # 2. get_scan() -> once opened, the instance holds the pointcloud
    points = scan_reader.get_points()
    print(f'points.shape: {points.shape}')  # (N x 4)

    # 4. open_label() -> returns semantic labels (N, )
    # labels are in the range of 0-259
    label = scan_reader.open_label(label_path)
    print(f'label.shape: {label.shape}')  # (N, )

    # 4.1 open_label with arguments -> returns semantic and instance labels (N, )
    sem_labels, inst_labels = scan_reader.open_label(
        label_path, return_instance=True)
    print(f'sem_labels.shape: {sem_labels.shape}')  # (N, )
    print(f'inst_labels.shape: {inst_labels.shape}')  # (N, )

    # 5. get_label() -> once opened, the instance holds the labels
    labels = scan_reader.get_labels()
    print(f'label.shape: {labels.shape}')  # (N, )

    # 5.1 get_label with arguments -> returns semantic and instance labels
    sem_labels, inst_labels = scan_reader.get_labels(return_instance=True)
    print(f'sem_labels.shape: {sem_labels.shape}')  # (N, )
    print(f'inst_labels.shape: {inst_labels.shape}')  # (N, )

    # Example usage of KITTIScanWriter
    points = np.random.rand(1000, 4)
    labels = np.random.randint(0, 19, 1000)  # (N, ) in range [0, 255]
    scan_writer.write_scan(points, path='./test.bin')
    scan_writer.write_label(labels, path='./test.label')
