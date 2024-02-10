import numpy as np
import open3d as o3d
import glob

def load_boxes_from_directory(directory):
    box_files = glob.glob(f"{directory}/scene0011_00-*.npy")
    return [np.load(file) for file in box_files]

def get_aabb(box):
    return np.min(box, axis=0), np.max(box, axis=0)

def boxes_overlap(aabb1, aabb2):
    for i in range(3):  # Check each dimension
        if aabb1[0][i] > aabb2[1][i] or aabb2[0][i] > aabb1[1][i]:
            return False
    return True

def filter_overlapping_boxes(boxes):
    overlap_groups = [-1] * len(boxes)  # -1 indicates no group assigned
    group_id = 0

    for i, box in enumerate(boxes):
        if overlap_groups[i] == -1:  # Only assign a new group to non-grouped boxes
            overlap_groups[i] = group_id
            aabb1 = get_aabb(box)
            for j in range(i+1, len(boxes)):
                if overlap_groups[j] == -1:
                    aabb2 = get_aabb(boxes[j])
                    if boxes_overlap(aabb1, aabb2):
                        overlap_groups[j] = group_id  # Assign the same group
            group_id += 1

    # Keep only the first box in each overlap group
    unique_group_ids = set(overlap_groups)
    non_overlapping_boxes = [boxes[i] for i in range(len(boxes)) if overlap_groups[i] in unique_group_ids]
    return non_overlapping_boxes

def visualize(scene, boxes):
    geometries = []
    geometries.append(scene)
    for box in boxes:
        bbox = o3d.geometry.OrientedBoundingBox.create_from_points(
            o3d.utility.Vector3dVector(box))
        bbox.color = (1, 0, 0)  # Red color
        geometries.append(bbox)

    o3d.visualization.draw_geometries(geometries)

if __name__ == "__main__":


    # Load the point cloud data from the npy files
    scene_file_path = '/Users/leechina/Coding/3DGCTR-Visualization/MainScene/data/scene0011_00.npy'
    directory = '/Users/leechina/Coding/3DGCTR-Visualization/MainScene/data/boxes'
    # Load the scene point cloud and bounding box data
    scene_data = np.load(scene_file_path)
    boxes = load_boxes_from_directory(directory)
    filtered_boxes = filter_overlapping_boxes(boxes)
    print(f"Number of boxes before filtering: {len(boxes)}")
    print(f"Number of boxes after filtering: {len(filtered_boxes)}")

    # Separate the xyz coordinates and the RGB color data for the scene
    xyz_scene = scene_data[:, :3]
    rgb_scene = scene_data[:, 3:] / 255.0  # Normalize the RGB values to [0, 1]
    
    # Create a PointCloud object for the scene
    scene_pcd = o3d.geometry.PointCloud()
    scene_pcd.points = o3d.utility.Vector3dVector(xyz_scene)
    scene_pcd.colors = o3d.utility.Vector3dVector(rgb_scene)

    # visualize
    visualize(scene_pcd, filtered_boxes)
    

