import numpy as np
import scipy.interpolate as interpolate
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.svm import SVC
from sklearn.metrics import accuracy_score
import joblib
import time
import csv

class TrackConeSimulator:
    def __init__(self, num_points=200):
        # We want a track ~40m long in the y-direction
        length = 40.0
        s = np.linspace(0, length, num_points)
        
        # Number of curves: 0 to 3 (allow straight tracks)
        num_curves = np.random.randint(0,4)
        
        # Straight line in y from 0 to 40
        y = s.copy()
        
        # Add up to three sine curves in x if num_curves > 0
        x = np.zeros_like(s)
        for _ in range(num_curves):
            amp = np.random.uniform(0.5, 2.0)  # amplitude of the curve
            freq = np.random.uniform(0.5, 1.5) # frequency factor
            phase = np.random.uniform(0, 2*np.pi)
            x += amp * np.sin((freq * s / length) * 2*np.pi + phase)

        # Fit spline
        tck, u = interpolate.splprep([x, y], s=0)
        self.tck = tck
        self.spline_func = lambda t: interpolate.splev(t, tck)

        # Precompute curvature for spacing
        self.precompute_curvature()

    def precompute_curvature(self):
        t_eval = np.linspace(0,1,200)
        dx, dy = interpolate.splev(t_eval, self.tck, der=1)
        ddx, ddy = interpolate.splev(t_eval, self.tck, der=2)
        
        denom = (dx**2 + dy**2)**1.5
        denom[denom == 0] = 1e-9
        curvature = np.abs(dx*ddy - dy*ddx) / denom
        self.max_curvature = np.max(curvature)
        if self.max_curvature < 1e-9:
            self.max_curvature = 1e-9

    def curvature_based_spacing(self, curvature):
        norm_curv = curvature / self.max_curvature
        min_space = 250
        max_space = 1000
        # spacing = min_space + (max_space - min_space)*(1 - norm_curv)
        spacing = min_space + (max_space - min_space)*(1 - norm_curv)
        return spacing

    def generate_cones(self, 
                   base_lateral_offset=1.5, 
                   lateral_noise=0.02, 
                   width_variation_scale=0.1):
        """
        Generate cones along the track.

        You can adjust the constants below to control behavior:
        -----------------------------------------------------
        # Cone spacing parameters are controlled by curvature_based_spacing() and spacing clamping (if any)
        
        # Cone Dropping / Section Removal Parameters:
        drop_probability = 0.1        # Probability to drop random single cones after generation
        section_drop_probability = 0.05 # Probability to drop a consecutive section of cones
        section_length_range = (2, 5)  # Range of consecutive cones to drop if section removal occurs
        
        # Lateral Offset Variation Parameters:
        # base_lateral_offset is passed into the function, so you can change it when calling
        # width_variation_scale is also passed; adjust it above in method signature as needed
        
        # Noise Parameters:
        # lateral_noise is passed in as parameter as well
        
        # Max Cones:
        max_total_cones = 50   # Maximum total cones (sum of left and right)
        # Since we place cones in pairs, we effectively have max_placements = max_total_cones // 2
        -----------------------------------------------------
        """
        
        # Define constants at the start so they are easy to tweak
        drop_probability = 0.1
        section_drop_probability = 0.05
        section_length_range = (2, 5)
        max_total_cones = 50  # total cones allowed
        max_placements = max_total_cones // 2

        # Evaluate spline and compute curvature
        t_eval = np.linspace(0,1,200)
        x_all, y_all = self.spline_func(t_eval)
        dx, dy = interpolate.splev(t_eval, self.tck, der=1)
        ddx, ddy = interpolate.splev(t_eval, self.tck, der=2)

        denom = (dx**2 + dy**2)**1.5
        denom[denom == 0] = 1e-9
        curvature = np.abs(dx*ddy - dy*ddx) / denom

        # Compute arc length along the spline
        ds = np.sqrt(dx**2 + dy**2)
        arc_length = np.cumsum(ds*(t_eval[1]-t_eval[0])*len(t_eval))
        total_length = arc_length[-1]

        def s_to_t(s_val):
            return np.interp(s_val, arc_length, t_eval)

        # Add width variation
        lateral_offset = base_lateral_offset + np.random.uniform(-width_variation_scale, width_variation_scale)

        left_cones = []
        right_cones = []

        placements = 0
        s_current = 0.0
        while s_current <= total_length and placements < max_placements:
            t_curr = s_to_t(s_current)
            curv_idx = np.argmin(np.abs(t_eval - t_curr))
            local_curv = curvature[curv_idx]
            spacing = self.curvature_based_spacing(local_curv)

            # If you want to enforce a fixed min/max spacing, do it here:
            # spacing = max(0.5, min(spacing, 5.0))

            x_pt, y_pt = self.spline_func(t_curr)
            dx_pt, dy_pt = interpolate.splev(t_curr, self.tck, der=1)
            norm_vec = np.array([-dy_pt, dx_pt])
            norm_len = np.linalg.norm(norm_vec)
            if norm_len < 1e-9:
                norm_len = 1e-9
            norm_vec /= norm_len

            lat_noise_left = np.random.normal(0, lateral_noise)
            left_cone = np.array([x_pt, y_pt]) + norm_vec * (lateral_offset + lat_noise_left)
            lat_noise_right = np.random.normal(0, lateral_noise)
            right_cone = np.array([x_pt, y_pt]) - norm_vec * (lateral_offset + lat_noise_right)

            left_cones.append(left_cone)
            right_cones.append(right_cone)

            placements += 1
            s_current += spacing

        left_cones = np.array(left_cones)
        right_cones = np.array(right_cones)

        # Now apply random dropping and section removal after generation
        # Drop random single cones with some probability
        if np.random.rand() < drop_probability:
            # drop a single cone from one side
            side = np.random.choice(['left', 'right'])
            if side == 'left' and len(left_cones) > 0:
                drop_idx = np.random.randint(len(left_cones))
                left_cones = np.delete(left_cones, drop_idx, axis=0)
            elif side == 'right' and len(right_cones) > 0:
                drop_idx = np.random.randint(len(right_cones))
                right_cones = np.delete(right_cones, drop_idx, axis=0)

        # Drop a consecutive section of cones with some probability
        if np.random.rand() < section_drop_probability:
            # Decide which side and how many to drop
            which_side = np.random.choice(['left', 'right', 'both'])
            section_length = np.random.randint(section_length_range[0], section_length_range[1] + 1)

            if which_side in ['left', 'both'] and len(left_cones) > section_length:
                start_idx = np.random.randint(0, max(len(left_cones)-section_length, 1))
                left_cones = np.delete(left_cones, slice(start_idx, start_idx+section_length), axis=0)
            if which_side in ['right', 'both'] and len(right_cones) > section_length:
                start_idx_r = np.random.randint(0, max(len(right_cones)-section_length, 1))
                right_cones = np.delete(right_cones, slice(start_idx_r, start_idx_r+section_length), axis=0)

        return left_cones, right_cones
    


    def visualize_sample_tracks(self, num_samples=3):
        fig, axes = plt.subplots(1, num_samples, figsize=(10, 4))
        if num_samples == 1:
            axes = [axes]

        for i in range(num_samples):
            self.__init__(num_points=200)
            left_cones, right_cones = self.generate_cones()
            
            t = np.linspace(0,1,200)
            track_x, track_y = self.spline_func(t)

            ax = axes[i]
            ax.plot(track_x, track_y, 'g-', label='Track Centerline')
            
            if len(left_cones) > 0:
                ax.scatter(left_cones[:,0], left_cones[:,1], color='blue', label='Left Cones')
            if len(right_cones) > 0:
                ax.scatter(right_cones[:,0], right_cones[:,1], color='red', label='Right Cones')

            ax.set_title(f'Sample Track {i+1}')
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.legend()
            ax.grid(True)

        plt.tight_layout()
        plt.show()

    def generate_training_data(self, 
                               num_samples=1000, 
                               num_splines=10, 
                               max_train_time=30.0):
        print("Starting data generation...")
        start_time = time.time()
        all_features = []
        all_labels = []

        spline_count = 0
        while spline_count < num_splines and (time.time() - start_time) < max_train_time:
            print(f"Generating data for spline {spline_count+1}/{num_splines}...")
            self.__init__(num_points=200)
            sample_count = 0
            while sample_count < num_samples and (time.time() - start_time) < max_train_time:
                left_cones, right_cones = self.generate_cones()
                
                if len(left_cones) == 0 and len(right_cones) == 0:
                    print("No cones generated this iteration, skipping...")
                    sample_count += 1
                    continue

                cones = np.vstack([left_cones, right_cones]) if (len(left_cones)>0 or len(right_cones)>0) else np.empty((0,2))
                labels = np.array([0]*len(left_cones) + [1]*len(right_cones))

                z_values = np.zeros((cones.shape[0], 1))
                cones_3d = np.hstack((cones, z_values))

                for cone_3d, label in zip(cones_3d, labels):
                    all_features.append(cone_3d)
                    all_labels.append(label)

                sample_count += 1
                if sample_count % 100 == 0:
                    print(f"  Generated {sample_count}/{num_samples} samples for spline {spline_count+1}...")
            spline_count += 1

        all_features = np.array(all_features)
        all_labels = np.array(all_labels)

        print("Finished data generation.")
        print(f"Total feature shape: {all_features.shape}, label shape: {all_labels.shape}")

        if np.isnan(all_features).any():
            print("Warning: NaN values found in features.")
        if all_features.shape[0] == 0:
            print("Warning: No data generated.")

        return all_features, all_labels

def train_cone_classifier(X, y, test_size=0.2):
    print("Starting training...")
    print("Splitting data...")
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=test_size, random_state=42)
    print(f"Train shape: {X_train.shape}, Test shape: {X_test.shape}")

    print("Scaling data...")
    scaler = StandardScaler()
    X_train_scaled = scaler.fit_transform(X_train)
    X_test_scaled = scaler.transform(X_test)

    if np.isnan(X_train_scaled).any():
        print("NaN values found in scaled training data.")
    if np.isnan(X_test_scaled).any():
        print("NaN values found in scaled test data.")

    print("Training SVM classifier...")
    classifier = SVC(kernel='rbf', C=1.0, gamma='scale', random_state=42)
    classifier.fit(X_train_scaled, y_train)

    print("Evaluating classifier...")
    y_pred = classifier.predict(X_test_scaled)
    accuracy = accuracy_score(y_test, y_pred)
    print(f"Training complete. Accuracy on test set: {accuracy:.4f}")

    print("Saving model and scaler...")
    joblib.dump(classifier, 'cone_classifier_svm.joblib')
    joblib.dump(scaler, 'scaler_svm.joblib')
    print("Model and scaler saved.")

    return classifier, scaler, accuracy

def test_on_real_data_unlabeled(classifier, scaler, csv_path, visualize=True):
    print(f"Testing (inference) on real data from {csv_path}...")
    features = []
    with open(csv_path, 'r') as f:
        reader = csv.reader(f)
        header = next(reader, None)
        print("Header:", header)

        for row in reader:
            if len(row) < 3:
                continue
            x_str, y_str, z_str = row
            try:
                x = float(x_str)
                y = float(y_str)
                z = float(z_str)
                features.append([x, y, z])
            except ValueError:
                continue

    features = np.array(features)
    print(f"Loaded {len(features)} samples from real data.")

    if len(features) == 0:
        print("No real data samples found. Cannot classify.")
        return None

    features_scaled = scaler.transform(features)
    pred = classifier.predict(features_scaled)

    print("Classification done. Visualizing results...")

    if visualize:
        left_mask = (pred == 0)
        right_mask = (pred == 1)

        plt.figure(figsize=(8,6))
        plt.scatter(features[left_mask, 0], features[left_mask, 1], color='blue', label='Predicted Left Cones', alpha=0.7)
        plt.scatter(features[right_mask, 0], features[right_mask, 1], color='red', label='Predicted Right Cones', alpha=0.7)

        plt.title('Visualization of Classified Cones (Unlabeled Real Data)')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.legend()
        plt.grid(True)
        plt.show()

    return pred

if __name__ == "__main__":
    simulator = TrackConeSimulator()
    
    print("Visualizing sample tracks...")
    simulator.visualize_sample_tracks(num_samples=3)

    print("Generating training data...")
    X, y = simulator.generate_training_data(num_samples=10, num_splines=500, max_train_time=30.0)
    
    if len(X) == 0:
        print("No data generated, cannot train.")
    else:
        classifier, scaler, accuracy = train_cone_classifier(X, y)
        print(f"Training Accuracy: {accuracy:.4f}")

        # Test on real data (unlabeled)
        test_on_real_data_unlabeled(classifier, scaler, 'test_SVM_straight.csv', visualize=True)