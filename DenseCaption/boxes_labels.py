import numpy as np
import open3d as o3d
import glob
import colorsys
import open3d.visualization.gui as gui


def create_text_marker(position, index, size=0.05, color=[1, 1, 1]):
    """
    Create a small point cloud at the given position to represent text.
    """
    marker = o3d.geometry.PointCloud()
    # Use a small offset so the marker does not intersect with the box edges
    offset = np.array([0, 0, size])
    marker.points = o3d.utility.Vector3dVector([position + offset])
    # Use the index to create a unique color for each text marker
    marker_color = generate_unique_color(index, 24)  # Assuming 24 boxes
    marker.colors = o3d.utility.Vector3dVector([marker_color])
    return marker

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
def load_boxes_from_directory(directory):
    box_files = glob.glob(f"{directory}/scene0496_00-gt-*.npy")
    boxes = []
    box_labels = []
    for file in box_files:
        # Load the box
        boxes.append(np.load(file))
        # Extract the last number from the file name
        label = file.split('-')[-1].split('.')[0]
        box_labels.append(label)
    return boxes, box_labels

def visualize(scene, boxes, box_labels):
    # Initialize the GUI application
    app = gui.Application.instance
    app.initialize()

    total_boxes = len(boxes)

    # Create a visualizer window
    vis = o3d.visualization.O3DVisualizer("Open3D - Box Labels", 1024, 768)
    vis.show_settings = True
    vis.add_geometry("Scene", scene)

    for index, (box,label) in enumerate(zip(boxes, box_labels)):
        box_corners = o3d.utility.Vector3dVector(box)
        edge_radius = 0.03
        edge_color = generate_unique_color(index, total_boxes)
        
        box_edges = create_box_edges_with_cylinders(box_corners, edge_radius, edge_color)
        
        for edge in box_edges:
            vis.add_geometry(f"Box{index}Edge", edge)
        
        # Calculate the center of the box for the label position
        box_center = np.mean(box, axis=0)
        #vis.add_3d_label(box_center, f"Box {label}")
        vis.add_3d_label(box_center, label)

    vis.reset_camera_to_default()
    app.add_window(vis)
    app.run()

if __name__ == "__main__":


    # Load the point cloud data from the npy files
    scene_file_path = '/Users/leechina/Coding/3DGCTR-Visualization/DenseCaption/data/scene0496_00.npy'
    directory = '/Users/leechina/Coding/3DGCTR-Visualization/DenseCaption/data/scene0496_gtboxes'
    # Load the scene point cloud and bounding box data
    scene_data = np.load(scene_file_path)
    boxes, box_labels = load_boxes_from_directory(directory)
    

    # Separate the xyz coordinates and the RGB color data for the scene
    xyz_scene = scene_data[:, :3]
    rgb_scene = scene_data[:, 3:] / 255.0  # Normalize the RGB values to [0, 1]
    
    # Create a PointCloud object for the scene
    scene_pcd = o3d.geometry.PointCloud()
    scene_pcd.points = o3d.utility.Vector3dVector(xyz_scene)
    scene_pcd.colors = o3d.utility.Vector3dVector(rgb_scene)

    # visualize
    visualize(scene_pcd, boxes, box_labels)
    

