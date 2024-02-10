import numpy as np
import open3d as o3d

'''
    input:
        p1:Vector3dVector
        p2:Vector3dVector
        radius:
        color:
    output:
'''
def create_cylinder_from_two_points(p1, p2, radius, color):
    """
    Create a cylinder mesh between two points.
    """
    vec = p2 - p1 # Calculates the vector from p1 to p2.
    length = np.linalg.norm(vec) # Computes the length of this vector, which is the distance between p1 and p2.
    vec /= length # Normalize the vector to get the unit vector

    # Create a cylinder mesh using the distance as the height
    cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=radius, height=length)
    cylinder.translate(p1 + vec * length / 2.0)# move the cylinder to the midpoint between p1 and p2
    # Align the cylinder with the vector from p1 to p2
    axis = np.array([0.0, 0.0, 1.6])
    rotation = o3d.geometry.get_rotation_matrix_from_axis_angle(np.cross(axis, vec))
    cylinder.rotate(rotation, center=cylinder.get_center())
    # Color the cylinder
    cylinder.paint_uniform_color(color)
    
    return cylinder

def create_box_edges_with_cylinders(box_data, radius, color):
    """
    Create a list of cylinder meshes that represent the edges of a bounding box.
    """
    
    edges = []
    # Define the pairs of points that form the edges of the bounding box
    edge_indices = [
        [0, 1], [0, 2], [1, 3], [2, 3],
        [4, 5], [4, 6], [5, 7], [6, 7],
        [0, 4], [1, 5], [2, 6], [3, 7],
    ]
    # Create a cylinder for each edge
    for edge in edge_indices:
        p1, p2 = box_data[edge[0]], box_data[edge[1]]
        cylinder = create_cylinder_from_two_points(p1, p2, radius, color)
        edges.append(cylinder)
    
    return edges

if __name__ == "__main__":

    # Paths to the data files
    scene_file_path = '/Users/leechina/Coding/3DGCTR-Visualization/VisualGrounding/VG/step333/pointcloud.pth.npy'
    #scene_file_path1 = '/Users/leechina/Coding/Open3D/3DGCTR/DC/DC-vote2cap/scene0496_00.npy'
    # Load the bounding box data
    box_eda_data = np.load('/Users/leechina/Coding/3DGCTR-Visualization/VisualGrounding/VG/step333/box_eda.pth.npy')  # Replace with your actual file path
    box_gt_data = np.load('/Users/leechina/Coding/3DGCTR-Visualization/VisualGrounding/VG/step333/box_gt.pth.npy')  # Replace with your actual file path
    box_dg_data = np.load('/Users/leechina/Coding/3DGCTR-Visualization/VisualGrounding/VG/step333/box_3dgctr.pth.npy')  # Replace with your actual file path
    scene_data = np.load(scene_file_path)
    # scene_data1 = np.load(scene_file_path1)
    # print(scene_data.shape)
    # print(scene_data[0])
    # exit()

    # Define the bounding box corner points
    bbox_eda_corners = o3d.utility.Vector3dVector(box_eda_data)
    bbox_dg_corners = o3d.utility.Vector3dVector(box_dg_data)
    bbox_gt_corners = o3d.utility.Vector3dVector(box_gt_data)

    # Define the radius for the cylinders
    edge_radius = 0.03  # Example radius, you may need to adjust this based on your data scale

    # Define the color for the cylinders (RGB)
    edge_eda_color = [0, 0, 1]  # Blue
    edge_dg_color = [1, 0, 0]  # Red
    edge_gt_color = [0, 1, 0]  # Green

    # Create the bounding box edges with cylinders
    bbox_eda_edges = create_box_edges_with_cylinders(bbox_eda_corners, edge_radius, edge_eda_color)
    bbox_dg_edges = create_box_edges_with_cylinders(bbox_dg_corners, edge_radius, edge_dg_color)
    bbox_gt_edges = create_box_edges_with_cylinders(bbox_gt_corners, edge_radius, edge_gt_color)
    
    # Separate the xyz coordinates and the RGB color data for the scene
    xyz_scene = scene_data[:, :3]
    rgb_scene = scene_data[:, 3:] / 255.0  # Normalize the RGB values to [0, 1]
    
    # Create a PointCloud object for the scene
    scene_pcd = o3d.geometry.PointCloud()
    scene_pcd.points = o3d.utility.Vector3dVector(xyz_scene)
    scene_pcd.colors = o3d.utility.Vector3dVector(rgb_scene)

    # Load your point cloud or other geometries here
    geometries = [scene_pcd]   + bbox_dg_edges + bbox_gt_edges +  bbox_eda_edges# Add other geometries you want to visualize

    # Visualize the scene and the bounding box edges
    o3d.visualization.draw_geometries(geometries)
