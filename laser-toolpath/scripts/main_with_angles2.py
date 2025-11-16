import pyvista as pv
import numpy as np
from sklearn.cluster import DBSCAN
from sklearn.cluster import KMeans
from sklearn.preprocessing import normalize
from python_tsp.heuristics import solve_tsp_simulated_annealing
from scipy.spatial.distance import cdist
import csv

# Load a CAD model
mesh = pv.read("/home/sahilsnair/Desktop/laser-toolpath/scripts/cad_models/new/curv_obj_1m.stl")
# mesh = pv.read("/home/sahilsnair/Desktop/laser-toolpath/scripts/cad_models/FemaleHead.stl")

# mesh = mesh.clip('z', invert=False,value=5)
mesh = mesh.clip('z', invert=False,value=0)

center = mesh.center
# print(center)
target_position = np.array([1200.0, 0.0,800.0])
translation_vector = target_position - center
mesh = mesh.translate(translation_vector, inplace=False)

mesh = mesh.compute_normals(cell_normals=True, point_normals=True)
cell_centers = mesh.cell_centers().points
cell_normals = mesh.cell_data['Normals']
point_normals = mesh.point_data['Normals']

normalized_normals = normalize(cell_normals)
features = np.hstack((cell_centers, normalized_normals))

n_clusters = 30
kmeans = KMeans(n_clusters=n_clusters, random_state=0)
clusters = kmeans.fit_predict(features)

centroids = kmeans.cluster_centers_

mesh.cell_data['Cluster'] = clusters

# Calculate area for each segment
segment_areas = []
for cluster_id in range(n_clusters):
    # Extract cells belonging to this cluster
    cluster_mask = clusters == cluster_id
    segment = mesh.extract_cells(cluster_mask)
    
    # Calculate area of this segment
    segment_area = segment.area
    segment_areas.append(segment_area)
    
    print(f"Segment {cluster_id} area: {segment_area:.2f}")

# Calculate total area of all segments
total_segment_area = sum(segment_areas)
print(f"\nTotal segments area: {total_segment_area:.2f}")
print(f"Original mesh area: {mesh.area:.2f}")
print(f"Coverage percentage: {(total_segment_area / mesh.area) * 100:.2f}%")
print(f"All individual segment areas{segment_areas} smaller than laser window 49 cm2")

# Project each centroid to the surface by finding the closest point on the mesh
projected_centroids = []
normals_at_centroids = []
for centroid in centroids[:, :3]:  # Only x, y, z coordinates
   # Find the index of the closest point on the mesh to the centroid
    closest_point_index = mesh.find_closest_point(centroid)
    # Retrieve the coordinates of that point
    closest_point_coords = mesh.points[closest_point_index]
    closest_point_normal = point_normals[closest_point_index]

    projected_centroids.append(closest_point_coords)
    normals_at_centroids.append(closest_point_normal)

# Convert the projected centroids list to a NumPy array
projected_centroids = np.array(projected_centroids)
normals_at_centroids = np.array(normals_at_centroids)

# Create a point cloud for the projected centroids on the surface
projected_centroid_cloud = pv.PolyData(projected_centroids)
projected_centroid_cloud.point_data['Cluster'] = np.arange(n_clusters)
projected_centroid_cloud.point_data['Normals'] = normals_at_centroids
# print(projected_centroids)

# Using tsp-solver to compute a path
distance_matrix = cdist(projected_centroids, projected_centroids, metric='euclidean')
permutation, distance = solve_tsp_simulated_annealing(distance_matrix)
print(f"Shortest path order: {permutation}")
print(f"Total distance: {distance}")
ordered_centroids = projected_centroids[permutation]
ordered_normals = normals_at_centroids[permutation]
print(ordered_centroids)

scale_factor = 0.001
combined_centroid_normals = [
    [round(float(c * scale_factor), 4) for c in centroid] + 
    [round(float(n), 4) for n in normal]
    for centroid, normal in zip(ordered_centroids, ordered_normals)
]

with open("/home/sahilsnair/Desktop/laser-toolpath/scripts/robotpath.csv", "w", newline="") as file:
    writer = csv.writer(file)
    writer.writerows(combined_centroid_normals)

plotter = pv.Plotter(off_screen=True)
# plotter = pv.Plotter()
plotter.add_mesh(mesh, scalars='Cluster', show_edges=True, cmap="viridis", label='Clusters')
plotter.add_mesh(projected_centroid_cloud, color='red', point_size=10, render_points_as_spheres=True, label='Projected Centroids on Surface')
plotter.add_arrows(projected_centroid_cloud.points, normals_at_centroids, mag=0.5, color='blue', label='Normals at Projected Centroids')
# plotter.save_graphic("images/curv_obj_0.5m.svg")
plotter.screenshot("images/curv_obj_1m.png")
plotter.show()