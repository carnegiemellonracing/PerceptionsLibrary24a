import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
import numpy as np

def parse_and_visualize_clusters(csv_file_path):
    """
    Parse a CSV file with x, y, z, intensity, and color information and visualize the clusters as blue or yellow.

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
    point_colors = data['color'].values  # 'color' column contains the assigned color (e.g., 'blue', 'yellow')

    # Split points by color
    blue_points = true_points[point_colors == 'blue']
    yellow_points = true_points[point_colors == 'yellow']

    # Visualize points
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim([0, 40])
    ax.set_ylim([-20, 20])
    ax.set_zlim([-15, 15])

    # Plot blue and yellow points
    ax.scatter(blue_points[:, 0], blue_points[:, 1], blue_points[:, 2], s=1, color='blue', label='Blue')
    ax.scatter(yellow_points[:, 0], yellow_points[:, 1], yellow_points[:, 2], s=1, color='red', label='Yellow')

    ax.legend()
    plt.show()

# Example Usage
parse_and_visualize_clusters('alex_shit/project/build/output.csv')