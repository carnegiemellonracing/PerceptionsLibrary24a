#include <cuda_runtime.h>
#include <vector>
#include <cmath>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/sort.h>
#include <thrust/partition.h>
#include <thrust/reduce.h>
#include <iostream>

#define NUM_BINS 25
#define KMEANS_MAX_ITER 100
#define KMEANS_EPSILON 1e-4
#define MAX_RADIUS 40.0f

struct Point {
    float x, y, z, intensity;

    __host__ __device__
    Point(float x = 0, float y = 0, float z = 0, float intensity = 0)
        : x(x), y(y), z(z), intensity(intensity) {}
};

// CUDA kernel to calculate the radius and assign bins
__global__ void assignBins(const Point* points, int* bin_indices, int num_points, float bin_size) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_points) return;

    float radius = sqrtf(points[idx].x * points[idx].x + points[idx].y * points[idx].y);
    bin_indices[idx] = min(static_cast<int>(radius / bin_size), NUM_BINS - 1);
}

// CUDA kernel for k-Means clustering (one iteration per call)
__global__ void kMeansUpdate(const Point* points, const int* bin_indices, const int* cluster_assignments,
                             float* cluster_sums, int* cluster_counts, int num_points, int num_bins) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_points) return;

    int bin_idx = bin_indices[idx];
    int cluster = cluster_assignments[idx];
    atomicAdd(&cluster_sums[bin_idx * 2 + cluster], points[idx].intensity);
    atomicAdd(&cluster_counts[bin_idx * 2 + cluster], 1);
}

// Host function for k-Means clustering within bins
std::vector<int> performKMeans(const std::vector<Point>& points, const std::vector<int>& bin_indices, int num_bins) {
    int num_points = points.size();
    thrust::device_vector<Point> d_points(points.begin(), points.end());
    thrust::device_vector<int> d_bin_indices(bin_indices.begin(), bin_indices.end());
    thrust::device_vector<int> d_cluster_assignments(num_points, 0);
    thrust::device_vector<float> d_cluster_means(num_bins * 2, 0.0f);

    // Initialize means to min and max intensities for each bin
    std::vector<float> initial_means(num_bins * 2, 0.0f);
    for (int b = 0; b < num_bins; ++b) {
        float min_intensity = INFINITY, max_intensity = -INFINITY;
        for (int i = 0; i < num_points; ++i) {
            if (bin_indices[i] == b) {
                min_intensity = fminf(min_intensity, points[i].intensity);
                max_intensity = fmaxf(max_intensity, points[i].intensity);
            }
        }
        initial_means[b * 2] = min_intensity;
        initial_means[b * 2 + 1] = max_intensity;
    }
    d_cluster_means = initial_means;

    thrust::device_vector<float> d_new_means(num_bins * 2, 0.0f);
    thrust::device_vector<int> d_cluster_counts(num_bins * 2, 0);

    for (int iter = 0; iter < KMEANS_MAX_ITER; ++iter) {
        thrust::fill(d_cluster_counts.begin(), d_cluster_counts.end(), 0);
        thrust::fill(d_new_means.begin(), d_new_means.end(), 0.0f);

        dim3 block(256);
        dim3 grid((num_points + block.x - 1) / block.x);

        kMeansUpdate<<<grid, block>>>(
            thrust::raw_pointer_cast(d_points.data()),
            thrust::raw_pointer_cast(d_bin_indices.data()),
            thrust::raw_pointer_cast(d_cluster_assignments.data()),
            thrust::raw_pointer_cast(d_new_means.data()),
            thrust::raw_pointer_cast(d_cluster_counts.data()),
            num_points, num_bins);

        // Update means
        for (int b = 0; b < num_bins; ++b) {
            for (int k = 0; k < 2; ++k) {
                int count = d_cluster_counts[b * 2 + k];
                if (count > 0) {
                    d_cluster_means[b * 2 + k] = d_new_means[b * 2 + k] / count;
                }
            }
        }
    }

     return std::vector<int>(d_cluster_assignments.begin(), d_cluster_assignments.end());
}



void printIntensitiesPerBin(const std::vector<Point>& points, const std::vector<int>& bin_indices, int num_bins) {
    // Create a container for each bin
    std::vector<std::vector<float>> bins(num_bins);

    // Group intensities by bin
    for (size_t i = 0; i < points.size(); ++i) {
        int bin = bin_indices[i];
        if (bin >= 0 && bin < num_bins) {
            bins[bin].push_back(points[i].intensity);
        }
    }

    // Print intensities for each bin
    for (int b = 0; b < num_bins; ++b) {
        std::cout << "Bin " << b << ": ";
        for (float intensity : bins[b]) {
            std::cout << intensity << " ";
        }
        std::cout << std::endl;
    }
}


// // Host function for color assignment
// std::vector<std::pair<Point, std::string>> ClusterCones(const std::vector<Point>& points) {
//     int num_points = points.size();
//     float bin_size = MAX_RADIUS / NUM_BINS;

//     thrust::host_vector<int> bin_indices(num_points);
//     thrust::device_vector<Point> d_points(points.begin(), points.end());
//     thrust::device_vector<int> d_bin_indices(num_points);

//     dim3 block(256);
//     dim3 grid((num_points + block.x - 1) / block.x);

//     assignBins<<<grid, block>>>(thrust::raw_pointer_cast(d_points.data()), thrust::raw_pointer_cast(d_bin_indices.data()), num_points, bin_size);
//     bin_indices = d_bin_indices;


//     // Convert thrust::host_vector<int> to std::vector<int>
//     std::vector<int> bin_indices_std(bin_indices.begin(), bin_indices.end());



//     // Print intensities per bin
//     printIntensitiesPerBin(points, bin_indices_std, NUM_BINS);


//     // Perform k-Means clustering
//     std::vector<int> cluster_assignments = performKMeans(points, bin_indices_std, NUM_BINS);

//     std::vector<std::pair<Point, std::string>> colored_points;
//     for (int i = 0; i < num_points; ++i) {
//         std::string color = (cluster_assignments[i] == 0) ? "Blue" : "Yellow"; // Assign color
//         colored_points.emplace_back(points[i], color); // Pair the point with the color
//     }

//     return colored_points;

// }




std::vector<std::pair<Point, std::string>> ClusterCones(const std::vector<Point>& points) {
    int num_points = points.size();
    float bin_size = MAX_RADIUS / NUM_BINS;

    thrust::host_vector<int> bin_indices(num_points);
    thrust::device_vector<Point> d_points(points.begin(), points.end());
    thrust::device_vector<int> d_bin_indices(num_points);

    dim3 block(256);
    dim3 grid((num_points + block.x - 1) / block.x);

    // Step 1: Assign bins
    assignBins<<<grid, block>>>(thrust::raw_pointer_cast(d_points.data()), thrust::raw_pointer_cast(d_bin_indices.data()), num_points, bin_size);
    bin_indices = d_bin_indices;

    // Convert thrust::host_vector<int> to std::vector<int>
    std::vector<int> bin_indices_std(bin_indices.begin(), bin_indices.end());

    // Step 2: Compute average intensity for each bin
    std::vector<float> bin_sums(NUM_BINS, 0.0f);
    std::vector<int> bin_counts(NUM_BINS, 0);

    for (int i = 0; i < num_points; ++i) {
        int bin = bin_indices_std[i];
        if (bin >= 0 && bin < NUM_BINS) {
            bin_sums[bin] += points[i].intensity;
            bin_counts[bin]++;
        }
    }

    std::vector<float> bin_averages(NUM_BINS, 0.0f);
    for (int b = 0; b < NUM_BINS; ++b) {
        if (bin_counts[b] > 0) {
            bin_averages[b] = bin_sums[b] / bin_counts[b];
        }
    }

    // Step 3: Assign colors based on average intensity
    std::vector<std::pair<Point, std::string>> colored_points;
    for (int i = 0; i < num_points; ++i) {
        int bin = bin_indices_std[i];
        if (bin >= 0 && bin < NUM_BINS) {
            std::string color = (points[i].intensity > bin_averages[bin]) ? "Yellow" : "Blue";
            colored_points.emplace_back(points[i], color);
        }
    }

    return colored_points;
}