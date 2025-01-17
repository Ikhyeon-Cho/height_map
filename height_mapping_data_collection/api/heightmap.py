#!/usr/bin/env python3

import numpy as np
import yaml
from pathlib import Path


class HeightMapReader:
    def __init__(self):
        self.grid_size = None
        self.resolution = None
        self.position = None
        self.layers = []

    def open_layer(self, layer: str, filename: str) -> np.ndarray:
        """Read data at layer"""
        if self.grid_size is None:
            self._read_map_info(str(Path(filename).parent))

        data = np.fromfile(filename + f'.{layer}', dtype=np.float32)
        return data.reshape(self.grid_size)

    def open_all(self, filename: str) -> np.ndarray:
        """Read all layers"""
        if self.grid_size is None:
            self._read_map_info(str(Path(filename).parent))

        # Initialize array with shape (H, W, n_layers)
        H, W = self.grid_size
        n_layers = len(self.layers)
        data_all = np.zeros((H, W, n_layers), dtype=np.float32)

        # Stack layers along the third dimension
        for i, layer in enumerate(self.layers):
            data = self.open_layer(layer, filename)
            data_all[:, :, i] = data

        return data_all

    def _read_map_info(self, folder_path: str):
        """Read grid parameters from yaml file"""

        # TODO: current map_info path is temporary. Fix this

        info_file = Path(folder_path) / 'map_info.yaml'
        if not info_file.exists():
            raise FileNotFoundError(f"Grid info file not found: {info_file}")

        with open(info_file, 'r') as f:
            config = yaml.safe_load(f)
            self.grid_size = config['size']
            self.resolution = config['resolution']
            self.position = config['position']
            self.layers = config['layers']


if __name__ == '__main__':

    map_reader = HeightMapReader()
    file = '/home/ikhyeon/ros/dev_ws/src/height_mapping/data/heightmap/000000'
    heightmap = map_reader.open_layer('elevation', file)

    # print(heightmap)
    print(heightmap.shape)

    heightmap_all = map_reader.open_all(file)

    # print(heightmap_all)
    print(heightmap_all.shape)
