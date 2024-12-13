import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# Top-level variables to control display
DISPLAY_MIDLINE = True
DISPLAY_SECTION_CIRCLES = True

def plot_circle(ax, center, radius, num_points=100):
    """
    Plot a circle in 3D at a given center with a specified radius.
    
    Args:
        ax: Matplotlib 3D axis.
        center (array-like): Center of the circle (x, y, z).
        radius (float): Radius of the circle.
        num_points (int): Number of points to approximate the circle.
    """
    theta = np.linspace(0, 2 * np.pi, num_points)
    x = center[0] + radius * np.cos(theta)
    y = center[1] + radius * np.sin(theta)
    z = np.full_like(x, center[2])  # Keep z constant
    ax.plot(x, y, z, color='black', alpha=0.5, label='Section Circle' if "Section Circle" not in ax.get_legend_handles_labels()[1] else "")

def parse_and_visualize_clusters(csv_file_path):
    """
    Parse a CSV file with x, y, z, intensity, and color information and visualize the clusters, midline points, and section circles.

    Args:
        csv_file_path (str): Path to the CSV file containing the point data.
    """
    # Read the CSV file
    data = pd.read_csv(csv_file_path)
    
    # Ensure required columns are present
    if not all(col in data.columns for col in ['x', 'y', 'z', 'intensity', 'color']):
        raise ValueError("CSV file must contain 'x', 'y', 'z', 'intensity', and 'color' columns.")
    
    # Extract points and colors
    true_points = data[['x', 'y', 'z']].values
    point_colors = data['color'].values  # 'color' column contains the assigned color (e.g., 'blue', 'yellow', 'green', 'section')

    # Split points by color
    blue_points = true_points[point_colors == 'blue']
    yellow_points = true_points[point_colors == 'yellow']
    green_points = true_points[point_colors == 'green']  # Midline points
    section_points = true_points[point_colors == 'section']  # Section start points

    # Visualize points
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim([0, 40])
    ax.set_ylim([-20, 20])
    ax.set_zlim([-15, 15])

    # Plot blue, yellow, and green points
    ax.scatter(blue_points[:, 0], blue_points[:, 1], blue_points[:, 2], s=1, color='blue', label='Blue (Track)')
    ax.scatter(yellow_points[:, 0], yellow_points[:, 1], yellow_points[:, 2], s=1, color='red', label='Yellow (Track)')

    if DISPLAY_MIDLINE:
        ax.scatter(green_points[:, 0], green_points[:, 1], green_points[:, 2], s=5, color='green', label='Green (Midline Points)')

    if DISPLAY_SECTION_CIRCLES:
        for section in section_points:
            plot_circle(ax, section, radius=5)

    ax.legend()
    plt.show()

# Example Usage
parse_and_visualize_clusters('project/build/output.csv')