import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import struct

class KITTIDataChecker:
    def __init__(self, data_path):
        self.data_path = Path(data_path)
        self.velodyne_path = self.data_path / "velodyne"
        self.heightmap_path = self.data_path / "heightmap"
        
    def read_layers(self):
        """Read layer names from layers.txt"""
        layer_file = self.heightmap_path / "layers.txt"
        with open(layer_file, 'r') as f:
            return f.read().splitlines()
            
    def read_heightmap(self, index):
        """Read height map and its invalid mask"""
        # Construct filenames
        data_file = self.heightmap_path / f"{index:06d}.bin"
        invalid_file = self.heightmap_path / f"{index:06d}.invalid"
        
        # Read binary data
        data = np.fromfile(str(data_file), dtype=np.float32)
        invalid = np.fromfile(str(invalid_file), dtype=np.uint8)
        
        # Get layer names
        layers = self.read_layers()
        num_layers = len(layers)
        
        # Calculate dimensions
        total_cells = len(invalid)
        height = int(np.sqrt(total_cells))  # Assuming square map
        width = height
        
        # Reshape arrays
        data = data.reshape(height, width, num_layers)
        invalid = invalid.reshape(height, width)
        
        return data, invalid, layers
        
    def read_pointcloud(self, index):
        """Read point cloud from velodyne binary file"""
        pc_file = self.velodyne_path / f"{index:06d}.bin"
        
        # Read binary data
        data = np.fromfile(str(pc_file), dtype=np.float32)
        
        # Reshape to (N, 4) - XYZI format
        points = data.reshape(-1, 4)
        
        return points
        
    def visualize_heightmap(self, index):
        """Visualize height map layers and invalid mask"""
        data, invalid, layers = self.read_heightmap(index)
        
        # Create subplot for each layer plus invalid mask
        n_plots = len(layers) + 1
        fig, axes = plt.subplots(1, n_plots, figsize=(5*n_plots, 4))
        
        # Plot each layer
        for i, layer in enumerate(layers):
            ax = axes[i]
            # Mask invalid cells
            masked_data = np.ma.array(data[:,:,i], mask=invalid)
            im = ax.imshow(masked_data, cmap='viridis')
            ax.set_title(f'Layer: {layer}')
            plt.colorbar(im, ax=ax)
            
        # Plot invalid mask
        ax = axes[-1]
        im = ax.imshow(invalid, cmap='gray')
        ax.set_title('Invalid Mask')
        plt.colorbar(im, ax=ax)
        
        plt.tight_layout()
        plt.show()
        
    def visualize_pointcloud(self, index):
        """Visualize point cloud"""
        points = self.read_pointcloud(index)
        
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        # Color points by height
        scatter = ax.scatter(points[:,0], points[:,1], points[:,2], 
                           c=points[:,2], cmap='viridis', s=1)
        
        plt.colorbar(scatter)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title(f'Point Cloud #{index:06d}')
        
        # Set equal aspect ratio
        max_range = np.array([points[:,0].max()-points[:,0].min(),
                            points[:,1].max()-points[:,1].min(),
                            points[:,2].max()-points[:,2].min()]).max() / 2.0
        mid_x = (points[:,0].max()+points[:,0].min()) * 0.5
        mid_y = (points[:,1].max()+points[:,1].min()) * 0.5
        mid_z = (points[:,2].max()+points[:,2].min()) * 0.5
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)
        
        plt.show()
        
    def print_statistics(self, index):
        """Print statistics about the data"""
        # Height map stats
        data, invalid, layers = self.read_heightmap(index)
        points = self.read_pointcloud(index)
        
        print(f"\nStatistics for frame #{index:06d}")
        print("\nHeight Map:")
        print(f"Dimensions: {data.shape}")
        print(f"Layers: {layers}")
        
        # Print some raw data values for debugging
        print("\nSample raw data values (first 5 cells):")
        for i, layer in enumerate(layers):
            print(f"Layer '{layer}': {data[:5, :5, i].flatten()}")
        
        for i, layer in enumerate(layers):
            layer_data = np.ma.array(data[:,:,i], mask=invalid)
            print(f"\nLayer '{layer}':")
            print(f"  Min: {layer_data.min()}")
            print(f"  Max: {layer_data.max()}")
            print(f"  Mean: {layer_data.mean()}")
            print(f"  Std: {layer_data.std()}")
        
        valid_percentage = 100 * (1 - invalid.mean())
        print(f"\nValid cells: {valid_percentage:.1f}%")
        
        print("\nPoint Cloud:")
        print(f"Number of points: {len(points)}")
        print(f"X range: [{points[:,0].min():.3f}, {points[:,0].max():.3f}]")
        print(f"Y range: [{points[:,1].min():.3f}, {points[:,1].max():.3f}]")
        print(f"Z range: [{points[:,2].min():.3f}, {points[:,2].max():.3f}]")
        print(f"Intensity range: [{points[:,3].min():.3f}, {points[:,3].max():.3f}]")

# Usage example
if __name__ == "__main__":
    # Replace with your data collection path
    data_path = "/home/ikhyeon/ros/dev_ws/src/height_mapping/height_mapping_ros/data"
    checker = KITTIDataChecker(data_path)
    
    # Check frame #0
    frame_idx = 0
    
    # Print statistics
    checker.print_statistics(frame_idx)
    
    # Visualize height map
    checker.visualize_heightmap(frame_idx)
    
    # Visualize point cloud
    checker.visualize_pointcloud(frame_idx)