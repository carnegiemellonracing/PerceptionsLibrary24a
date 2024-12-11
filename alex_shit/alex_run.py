'''
Evaluation code

    Base algorithm: Filtering algorithm + Clustering using DBScan 
    Filtering - Grace and Conrad, Intensity Based evaluation 
    
    Evaluated based on accuracy and runtime

'''
import math
import numpy as np 
import matplotlib.pyplot as plt 
from sklearn import cluster
import time
import csv 
import os

### Filtering Algorithm - G&C ###

'''
Data preprocessing - remove points clouds outside given FOV range
    - PC is only PC[:, :3] 
    - Remove radial point 0 - 40 
'''

def fov_range(pointcloud, minradius=0, maxradius=40):
    # Calculate radial distance of each point (straight line distance from origin) and removes if outside range
    # made 4 for intensity based 
    pointcloud = pointcloud[:,:4]
    points_radius = np.sqrt(np.sum(pointcloud[:,:2] ** 2, axis=1))

    # Uses mask to remove points to only store in-range points
    radius_mask = np.logical_and(
        minradius <= points_radius, 
        points_radius <= maxradius
    )
    pointcloud = pointcloud[radius_mask]
    return pointcloud


'''
G&C Filtering Algorithm (i.e. Ground Truth)
    - Split point into M segments based on angle 
    - Split each segment into rbins based on radial distance of each point 
    - Fit a line to each segment 
    - Any point below the line is the 'ground' and is filtered
'''

def grace_and_conrad_filtering(points):
    
    alpha = 0.1
    num_bins = 10
    height_threshold = 0.13

    
    # change so that take lowest x points - averga eand fit a plane across all segments 

    angles = np.arctan2(points[:, 1], points[:, 0])  # Calculate angle for each point
    bangles = np.where(angles < 0, angles + 2 * np.pi, angles)

    # NOTE: making gangles from min to max to avoid iterating over empty regions
    if (bangles.size > 0): 
        
        gangles = np.arange(np.min(bangles), np.max(bangles), alpha)

        # Map angles to segments
        segments = np.digitize(bangles, gangles) - 1 
        # Calculate range for each point
        ranges = np.sqrt(points[:, 0]**2 + points[:, 1]**2) 

        rmax = np.max(ranges)
        rmin = np.min(ranges)
        bin_size = (rmax - rmin) / num_bins
        rbins = np.arange(rmin, rmax, bin_size)
        regments = np.digitize(ranges, rbins) - 1

        M, N = len(gangles), len(rbins)
        grid_cell_indices = segments * N + regments

        gracebrace = []
        for seg_idx in range(M):
            Bines = []
            min_zs = []
            prev_z = None
            for range_idx in range(N):
                bin_idx = seg_idx * N + range_idx
                idxs = np.where(grid_cell_indices == bin_idx)
                bin = points[idxs, :][0]
                if bin.size > 0:
                    min_z = np.min(bin[:, 2])
                    binLP = bin[bin[:, 2] == min_z][0].tolist()
                    min_zs.append(min_z)
                    Bines.append([np.sqrt(binLP[0]**2 + binLP[1]**2), binLP[2]])
                    prev_z = min_z

            if Bines:
                i = 0
                while i < len(min_zs):
                    good_before = i == 0 or min_zs[i] - min_zs[i - 1] < 0.1
                    good_after = i == len(min_zs) - 1 or min_zs[i] - min_zs[i + 1] < 0.1
                    if not (good_before and good_after):
                        Bines.pop(i)
                        min_zs.pop(i)
                        i -= 1
                    i += 1

                seg = segments == seg_idx
                X = [p[0] for p in Bines]
                Y = [p[1] for p in Bines]
                
                X = np.array(X)
                Y = np.array(Y)

                x_bar = np.mean(X)
                y_bar = np.mean(Y)
                x_dev = X - x_bar
                y_dev = Y - y_bar
                ss = np.sum(x_dev * x_dev)

                slope = np.sum(x_dev * y_dev) / np.sum(x_dev * x_dev) if ss != 0 else 0
                intercept = y_bar - slope * x_bar
                
                points_seg = points[seg]
                pc_compare = slope * np.sqrt(points_seg[:, 0]**2 + points_seg[:, 1]**2) + intercept
                pc_mask = (pc_compare + height_threshold) < points_seg[:, 2]
                conradbonrad = points_seg[pc_mask]
                if conradbonrad.tolist(): gracebrace.extend(conradbonrad.tolist())
     
        gracebrace = np.array(gracebrace)
        return gracebrace.reshape((-1, 3))


### Clustering Algorithm - HDBSCAN ###

'''
HDBSCAN Clustering Algorithm 
    - Performs clustering using HDBSCAN 
    - Predicts cone centers 
    - Outputs (x,y,z) center position of each cone
'''

'''
    Runs HDBSCAN on points
    clusterer.labels_ - each element is the cluster label. Noisy points assigned -1
'''
def run_dbscan(points, eps=0.5, min_samples=1):

    start_time = time.time()


    clusterer = cluster.DBSCAN(eps=eps, min_samples=min_samples)
    clusterer.fit(points)


    end_time = time.time()

    elapsed_time_ms = (end_time - start_time) * 1000
    print(f"Execution time for DBSCAN: {elapsed_time_ms:.3f} ms")

    return clusterer

'''
    Returns the centroid for each cluster 
    Note: no additional filtering performed on size of cluster - i.e. using radial distance or ground plane
'''
def get_centroids_z(points, labels, scalar=1):
  
    points = np.zeros(points.shape) + points[:, :3]
    # Scales z-axis by scalar 
    points[:, 2] *= scalar
    
    
    n_clusters = np.max(labels) + 1
    centroids = []

    # Default probability of 1 to each point 
    probs = np.ones((points.shape[0], 1))

    # Iterate over each cluster 
    for i in range(n_clusters):
      
        # Extract points that belong to n_clusters[i]
        idxs = np.where(labels == i)[0]
        cluster_points = points[idxs]

        # Weighted average center for each cluster of points 
        cluster_probs = probs[idxs]
        scale = np.sum(cluster_probs)
        center = np.sum(cluster_points * cluster_probs, axis=0) / scale

        centroids.append(center)
        
        # NOTE: No additional filtering performed on size of cluster - i.e. using radial distance or ground plane
       

    return np.array(centroids)

'''
    Main code for cone clustering
'''
def predict_cones_z(points): 
    
    if points.shape[0] == 0:
        return np.zeros((0, 3))

    points = points[:, :3]
    zmax = (points[:, 2] + 1).max(axis=0)
    endscal = (abs(zmax))
    points[:, 2] /= endscal

    # Run DBSCAN - returns probabilities 
    clusterer = run_dbscan(points, min_samples=2, eps=0.3)
    labels = clusterer.labels_.reshape((-1, 1))

    # Extracts centroids for each cluster 
    centroids = get_centroids_z(points, labels)

    return centroids.reshape((-1, 3))


def intensity_filtering(points, max_dist=40):
    
    print("BEFORE CONES", points.shape)
    

    points = points[~np.all(points[:, :3] == [0, 0, 0], axis=1)]

    #array of r
    r = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
    #range mask
    mask_close = (r < 10)
    mask_far = (r > 10) & (r < 30)

    # Split the data into 2 arrays based on the masks
    close_points = points[mask_close]     # Points with 0 < r < 10
    far_points = points[mask_far]   # Points with 10 < r < 50
    far_r = np.sqrt(far_points[:, 0]**2 + far_points[:, 1]**2)

    #far points ground removal & transformation
    r_mask = (far_points[:, 3] * far_r * np.log(far_r) >= 200)
    far_points[~r_mask, :3] = 0
    far_points = far_points[:, :3]
    far_points = far_points[:, [1, 0, 2]] 
    far_points[:, 0] = -far_points[:, 0] 
    far_points = far_points[~np.all(far_points == 0, axis=1)]


    #close points removal & transformation
    threshold = .1
    close_points = close_points[:, :3]
    close_points = close_points[:, [1, 0, 2]] 
    close_points[:, 0] = -close_points[:, 0] 
    random_selection = close_points[np.random.choice(close_points.shape[0], 100, replace=False)]
    sorted_selection = random_selection[random_selection[:, 2].argsort()]
    remaining_points = sorted_selection[:-5]
    lowest_z_points = remaining_points
    X = lowest_z_points[:, 0]  
    Y = lowest_z_points[:, 1]  
    Z = lowest_z_points[:, 2] 
    A = np.vstack([X, Y, np.ones_like(X)]).T  #
    B = Z  
    coeffs, _, _, _ = np.linalg.lstsq(A, B, rcond=None)
    a, b, d = coeffs

    plane_z_values = close_points[:, 0] * a + close_points[:, 1] * b + d
    plane_mask = close_points[:, 2] >= plane_z_values + threshold #
    close_points = close_points[plane_mask]

    points = np.concatenate((close_points, far_points), axis=0)
    points = points[:, [1, 0, 2]]
    points[:, 1] = -points[:, 1]
    
    print("AFTER CONES", points.shape)
    
    return points

### MAIN RUN ###

def visualise(points):
    
        # with np.load(path, allow_pickle=True) as data:
        # frames = fov_range(data['points'], 0, 40)
        
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim([0, 40])
    ax.set_ylim([-20, 20])
    ax.set_zlim([-3, 3])
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], s=1)
    plt.show()


# def parse_csv_file(file_path):
#     with open(file_path, mode='r') as file:
#         csvFile = csv.reader(file)
#         all_lines = []
#         for line in csvFile: 
#             if line[0] != 'x' and len(line) == 3:
#                 all_lines.append(line)
            
#     raw_points = []
#     for i in range(len(all_lines)):
#         line = [float(s) for s in all_lines[i]]
#         # line = np.array(line)
#         # line = line.transpose([1,0,2])
#         raw_points.append(line)
    
#     return np.array(raw_points)


# points only
def parse_csv_file(file_path):
    with open(file_path, mode='r') as file:
        csvFile = csv.reader(file)
        all_lines = []
        for line in csvFile:
            # Ensure the line has at least 3 numeric values (ignoring header and extra columns)
            if line[0] != 'x' and len(line) >= 3:
                # Parse x, y, z only
                x, y, z = float(line[0]), float(line[1]), float(line[2])
                all_lines.append([x, y, z])
    
    # Convert the collected points to a numpy array
    return np.array(all_lines)


print(os.getcwd())



file_path = 'alex_shit/point_clouds/point_cloud_73.csv'
# FOR AFS
# file_path = 'point_clouds/intensity_1.csv'


raw_point_cloud = parse_csv_file(file_path)
processed_data_frame = fov_range(raw_point_cloud, 0, 40)


ground_filtered_points = grace_and_conrad_filtering(processed_data_frame)

# file_path = 'alex_shit/project/build/output.csv'

# clusters = parse_csv_file(file_path)


clusters = predict_cones_z(ground_filtered_points)





visualise(clusters)

