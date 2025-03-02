import numpy as np
import random
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

# Visualize the reconstructed surface
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# ax.scatter(points[:, 0], points[:, 1], points[:, 2], c='b', marker='o', label='Contact Points')

# Plot the triangles
ax.plot_trisurf(points[:, 0], points[:, 1], points[:, 2], triangles=tri.simplices, cmap='viridis')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()
plt.show()