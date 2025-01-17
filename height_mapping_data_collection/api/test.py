import numpy as np

data = np.fromfile(
    '/home/ikhyeon/ros/dev_ws/src/height_mapping/data/heightmap/000000.bin.elevation', dtype=np.float32)
# print(data.shape)

data = data.reshape(151, 151)

# print(data)
print(data[0, 75])
print(data[75, 0])
print(data[75, 75])
print(data[0, 0])
