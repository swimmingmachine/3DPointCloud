import numpy as np
import random
import open3d as o3d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import Delaunay

def simulate_contact_points(num_points=200, radius=10):
    """Simulates contact points on a hemispherical surface."""

    contact_points = []
    for _ in range(num_points):
        # Generate random spherical coordinates (hemisphere)
        phi = np.random.uniform(0, np.pi / 2)  # Hemisphere (0 to pi/2)
        theta = np.random.uniform(0, 2 * np.pi)

        # Convert to Cartesian coordinates
        x = radius * np.sin(phi) * np.cos(theta)
        y = radius * np.sin(phi) * np.sin(theta)
        z = radius * np.cos(phi)

        # Simulate electrical data (voltage, activation time)
        voltage = random.uniform(-5, 5)
        activation_time = random.uniform(0, 100)

        contact_points.append({
            'x': x,
            'y': y,
            'z': z,
            'voltage': voltage,
            'activation_time': activation_time,
        })

    return contact_points

# Generate simulated contact points
contact_data = simulate_contact_points()

# Extract x, y, z coordinates
points = np.array([[point['x'], point['y'], point['z']] for point in contact_data])

# Perform Delaunay Triangulation
tri = Delaunay(points)

# Perform Poisson Surface Reconstruction
# Create Open3D PointCloud
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)

# Estimate normals (important for Poisson reconstruction)
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30))

# Poisson Surface Reconstruction
mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9) #adjust depth for resolution.

# Visualize the reconstructed mesh
# o3d.visualization.draw_geometries([mesh])

#Optional Matplotlib Visualization
vertices = np.asarray(mesh.vertices)
triangles = np.asarray(mesh.triangles)

# Visualize the reconstructed surface
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# ax.scatter(points[:, 0], points[:, 1], points[:, 2], c='b', marker='o', label='Contact Points')

# Plot the triangles
ax.plot_trisurf(points[:, 0], points[:, 1], points[:, 2], triangles=tri.simplices, cmap='viridis')

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot_trisurf(vertices[:, 0], vertices[:, 1], vertices[:, 2], triangles=triangles, cmap='viridis')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()
plt.show()