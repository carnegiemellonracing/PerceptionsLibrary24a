import numpy as np
import pandas as pd
import sys


def GraceAndConrad (points, alpha, num_bins, height_threshold):

    angles = np.arctan2(points[:, 1], points[:, 0])  # Calculate angle for each point
    bangles = np.where(angles < 0, angles + 2 * np.pi, angles)

    # NOTE: making gangles from min to max to avoid iterating over empty regions
    if (bangles != []): 
        
        gangles = np.arange(np.min(bangles), np.max(bangles), alpha)

        segments = np.digitize(bangles, gangles) - 1 # Map angles to segments
        ranges = np.sqrt(points[:, 0]**2 + points[:, 1]**2)  # Calculate range for each point

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
            Lines = []
            Points = []
            c = 0
            if Bines:
                # NOTE: could make some of this faster with numpy - but short anyways so its fine
                filtered_Bines = []
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
                #res = stats.linregress(Bines)
                X = [p[0] for p in Bines]
                Y = [p[1] for p in Bines]

                

                # NOTE: perform linear regression (our own implementation for speed)
                X = np.array(X)
                Y = np.array(Y)

                x_bar = np.mean(X)
                y_bar = np.mean(Y)
                x_dev = X - x_bar
                y_dev = Y - y_bar
                ss = np.sum(x_dev * x_dev)

                slope = np.sum(x_dev * y_dev) / np.sum(x_dev * x_dev) if ss != 0 else 0
                intercept = y_bar - slope * x_bar

               

                # NOTE: calculating heights only on points within segment 
                
                points_seg = points[seg]
                pc_compare = slope * np.sqrt(points_seg[:, 0]**2 + points_seg[:, 1]**2) + intercept
                pc_mask = (pc_compare + height_threshold) < points_seg[:, 2]
                conradbonrad = points_seg[pc_mask]
                if conradbonrad.tolist(): gracebrace.extend(conradbonrad.tolist())

        gracebrace = np.array(gracebrace)
        return gracebrace.reshape((-1, 3))
    

# Wrapper function
def process_csv(input_csv, output_csv, alpha, num_bins, height_threshold):
    # Load points from the input CSV (account for the header row)
    df = pd.read_csv(input_csv)
    points = df[['x', 'y', 'z']].values  # Extract XYZ values as numpy array

    print(points[:5])

    # Ground-truth points not provided; use None as placeholder

    # Run GraceAndConrad
    result_points = GraceAndConrad(points, alpha, num_bins, height_threshold)

    # Save the filtered points to the output CSV
    result_df = pd.DataFrame(result_points, columns=['x', 'y', 'z'])
    result_df.to_csv(output_csv, index=False)

# Main function
if __name__ == "__main__":
    
    input_csv = "alex_shit/point_clouds/point_cloud_120.csv"
    output_csv = "grace_output.csv"
    alpha = 10
    num_bins = 10
    height_threshold = 0.5

    try:
        process_csv(input_csv, output_csv, alpha, num_bins, height_threshold)
        print(f"Processed points saved to {output_csv}")
    except Exception as e:
        print(f"Error: {e}")

