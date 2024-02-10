import numpy as np
import open3d as o3d
import glob
import colorsys



def generate_unique_color(index, total_boxes):
    """
    Generate a unique color for each box based on its index using HSL color space.
    """
    # Divide the color wheel into 24 segments
    hue = (index / 24) % 1.0
    saturation = 1.0
    lightness = 0.5
    # Convert HSL to RGB
    rgb_color = colorsys.hls_to_rgb(hue, lightness, saturation)
    return rgb_color


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


def load_boxes_from_directory(directory):
    box_files = glob.glob(f"{directory}/scene0011_00-*.npy")
    boxes = [np.load(file) for file in box_files]
    
    return boxes

def add_noise(boxes):
    noise_boxes = boxes
    # # Add noise to a random coordinate (x, y, or z) of each box
    for box in noise_boxes:
        # coordinate_index = np.random.choice([0, 1, 2])  # Randomly choose between x (0), y (1), or z (2)
        # noise = np.random.normal(0, 0.05, box[:, coordinate_index].shape)  # Generate noise
        # box[:, coordinate_index] += noise  # Add noise to the chosen coordinate
        direction_index = np.random.choice([0, 1, 2])  # Randomly choose a direction (x, y, or z)
        shift_value = np.random.normal(0, 0.1)  # Generate a noise value for the shift
        box[:, direction_index] += shift_value  # Shift the entire box in the chosen direction

    return noise_boxes
    

def visualize(scene, boxes, random_color=False):
    geometries = []
    geometries.append(scene)
    total_boxes = len(boxes)
    for index, box in enumerate(boxes):
        box_corners = o3d.utility.Vector3dVector(box)
        edge_radius = 0.03

        edge_color = [1, 0, 0] # Red color
        # Generate a unique color for each set of box edges
        if random_color:
            edge_color = generate_unique_color(index, total_boxes)
        
        box_edges = create_box_edges_with_cylinders(box_corners, edge_radius, edge_color)
        geometries.extend(box_edges)


    o3d.visualization.draw_geometries(geometries)

if __name__ == "__main__":


    # Load the point cloud data from the npy files
    scene_file_path = '/Users/leechina/Coding/3DGCTR-Visualization/MainScene/data/scene.npy'
    directory = '/Users/leechina/Coding/3DGCTR-Visualization/MainScene/data/boxes'
    # Load the scene point cloud and bounding box data
    scene_data = np.load(scene_file_path)
    boxes = load_boxes_from_directory(directory)
    #noise_boxes = add_noise(boxes)
    
    
    # Separate the xyz coordinates and the RGB color data for the scene
    xyz_scene = scene_data[:, :3]
    rgb_scene = scene_data[:, 3:] / 255.0  # Normalize the RGB values to [0, 1]
    
    # Create a PointCloud object for the scene
    scene_pcd = o3d.geometry.PointCloud()
    scene_pcd.points = o3d.utility.Vector3dVector(xyz_scene)
    scene_pcd.colors = o3d.utility.Vector3dVector(rgb_scene)

    # visualize
    visualize(scene_pcd, boxes, random_color=True)
    

