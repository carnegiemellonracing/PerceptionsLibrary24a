import numpy as np
import pandas as pd

# Load the .npz file
file_path = "alex_shit/point_clouds/instance-3.npz"
data = np.load(file_path)

with np.load(file_path, allow_pickle=True) as data:
    # Extract the 'points' array
    points = data['points']

    # Ensure the array has at least 5 columns for x, y, z, and intensity
    if points.shape[1] > 4:
        x, y, z, intensity = points[:, 0], points[:, 1], points[:, 2], points[:, 3]

        # Create a DataFrame
        df = pd.DataFrame({
            'x': x,
            'y': y,
            'z': z,
            'intensity': intensity
        })

        df = df[(df[['x', 'y', 'z', 'intensity']] != 0).any(axis=1)]


        # Save to CSV
        csv_path = "alex_shit/point_clouds/intensity_2.csv"
        df.to_csv(csv_path, index=False)
        print(f"CSV file saved at {csv_path}")
    else:
        print("The 'points' array does not have enough columns to extract x, y, z, and intensity.")