import numpy as np
import open3d as o3d

if __name__ == "__main__":

    # Paths to the data files
    scene_file_path = '/Users/leechina/Coding/3DGCTR-Visualization/MainScene/data/scene0011_00.npy'
    scene_data = np.load(scene_file_path)
    
    # Separate the xyz coordinates and the RGB color data for the scene
    xyz_scene = scene_data[:, :3]
    rgb_scene = scene_data[:, 3:] / 255.0  # Normalize the RGB values to [0, 1]
    
    # Create a PointCloud object for the scene
    scene_pcd = o3d.geometry.PointCloud()
    scene_pcd.points = o3d.utility.Vector3dVector(xyz_scene)
    scene_pcd.colors = o3d.utility.Vector3dVector(rgb_scene)

    # Load your point cloud or other geometries here
    geometries = [scene_pcd] 

    # Visualize the scene and the bounding box edges
    o3d.visualization.draw_geometries(geometries)
