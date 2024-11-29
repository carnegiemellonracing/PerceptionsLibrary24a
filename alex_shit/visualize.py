import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def visualise_3(true_points):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim([0, 40])
    ax.set_ylim([-20, 20])
    ax.set_zlim([-15, 15])
    ax.scatter(true_points[:, 0], true_points[:, 1], true_points[:, 2], s=1)
    plt.show()

def visualize_point_cloud(csv_file, x_col='x', y_col='y', z_col='z'):
    # Load the CSV into a DataFrame
    data = pd.read_csv(csv_file)
    
    # Check if required columns are present
    if not all(col in data.columns for col in [x_col, y_col, z_col]):
        raise ValueError(f"CSV must contain columns: {x_col}, {y_col}, {z_col}")
    
    # Extract the x, y, z coordinates as a NumPy array
    true_points = data[[x_col, y_col, z_col]].to_numpy()
    
    # Use the visualise_3 function for plotting
    visualise_3(true_points)



csv_file_path = 'alex_shit/project/build/output.csv'
visualize_point_cloud(csv_file_path)