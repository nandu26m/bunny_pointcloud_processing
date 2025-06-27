import open3d as o3d
import os

# Define path to the point cloud file
pcd_path = "pointcloud_project/data/bunny.ply"

# Check if file exists before loading
if not os.path.isfile(pcd_path):
    raise FileNotFoundError(f"Point cloud file not found: {pcd_path}")

# Load the point cloud
pcd = o3d.io.read_point_cloud(pcd_path)
print("Point cloud loaded successfully!")
print(f"Number of points: {len(pcd.points)}")

# Print bounding box extent
bounds = pcd.get_axis_aligned_bounding_box()
print(f"Bounding box extent: {bounds.get_extent()}")

# Visualize original point cloud
o3d.visualization.draw_geometries([pcd], window_name="Original Bunny Point Cloud")

# 1. Remove outliers
pcd_clean, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
print(f"Removed outliers, remaining points: {len(pcd_clean.points)}")

# Visualize cleaned point cloud
o3d.visualization.draw_geometries([pcd_clean], window_name="Cleaned Point Cloud")

# 2. Downsample the point cloud
pcd_down = pcd_clean.voxel_down_sample(voxel_size=0.005)
print(f"Downsampled point cloud points: {len(pcd_down.points)}")

# Visualize downsampled cloud
o3d.visualization.draw_geometries([pcd_down], window_name="Downsampled Point Cloud")

# 3. Estimate normals (needed for surface reconstruction)
pcd_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
print("Normals estimated.")

# 4. Plane segmentation (example: extract largest plane)
plane_model, inliers = pcd_down.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
print(f"Plane model: {plane_model}")
print(f"Number of inliers (plane points): {len(inliers)}")

inlier_cloud = pcd_down.select_by_index(inliers)
outlier_cloud = pcd_down.select_by_index(inliers, invert=True)

# Visualize plane vs rest
o3d.visualization.draw_geometries([inlier_cloud.paint_uniform_color([1, 0, 0]),  # red plane
                                   outlier_cloud.paint_uniform_color([0, 1, 0])], # green other points
                                  window_name="Plane Segmentation")

# 5. Surface reconstruction (Poisson)
# Use the downsampled cloud with normals
mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd_down, depth=9)
print("Poisson mesh created.")

# Visualize mesh
o3d.visualization.draw_geometries([mesh], window_name="Poisson Surface Reconstruction")

# 6. ICP registration example — you need two point clouds: source and target
# Here is a placeholder example assuming you have two clouds:

# source = o3d.io.read_point_cloud("path_to_source.ply")
# target = o3d.io.read_point_cloud("path_to_target.ply")

# Example (commented out until you have real data):
# transformation = o3d.pipelines.registration.registration_icp(
#     source, target, max_correspondence_distance=0.02,
#     estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint()
# ).transformation
# print("ICP Transformation:\n", transformation)

# 7. Color the cleaned point cloud orange
pcd_clean.paint_uniform_color([1, 0.706, 0])

# Visualize colored cleaned point cloud
o3d.visualization.draw_geometries([pcd_clean], window_name="Colored Cleaned Point Cloud")
