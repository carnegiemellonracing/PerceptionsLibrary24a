// #include <cuda_runtime.h>
// #include <vector>
// #include <iostream>
// #include <cmath>
// #include <thrust/device_vector.h>
// #include <thrust/host_vector.h>

// #define NUM_BINS 10
// #define MAX_DISTANCE 40.0f

// struct Point {
//     float x, y, z, intensity;

//     __host__ __device__
//     Point(float x = 0, float y = 0, float z = 0, float intensity = 0)
//         : x(x), y(y), z(z), intensity(intensity) {}
// };

// // Kernel to bin points by radius
// __global__ void binPoints(const Point* points, int* bin_indices, int num_points, float bin_size) {
//     int idx = blockIdx.x * blockDim.x + threadIdx.x;
//     if (idx >= num_points) return;

//     float range = sqrtf(points[idx].x * points[idx].x + points[idx].y * points[idx].y);
//     bin_indices[idx] = min(static_cast<int>(range / bin_size), NUM_BINS - 1);
// }

// // Kernel to compute average intensity for bins
// __global__ void computeBinAverages(const Point* points, const int* bin_indices, float* bin_avgs, int* bin_counts, int num_points) {
//     int idx = blockIdx.x * blockDim.x + threadIdx.x;
//     if (idx >= num_points) return;

//     int bin_idx = bin_indices[idx];
//     atomicAdd(&bin_avgs[bin_idx], points[idx].intensity);
//     atomicAdd(&bin_counts[bin_idx], 1);
// }

// // Kernel to classify points within bins
// __global__ void classifyPoints(const Point* points, const int* bin_indices, float* bin_avgs, int* labels, int num_points) {
//     int idx = blockIdx.x * blockDim.x + threadIdx.x;
//     if (idx >= num_points) return;

//     int bin_idx = bin_indices[idx];
//     labels[idx] = points[idx].intensity <= bin_avgs[bin_idx] ? 0 : 1; // 0 = Blue, 1 = Yellow
// }

// // Function to process points and assign colors
// std::vector<std::pair<Point, std::string>> ClusterCones(const std::vector<Point>& points) {
//     int num_points = points.size();
//     float bin_size = MAX_DISTANCE / NUM_BINS;

//     // Allocate device memory
//     thrust::device_vector<Point> d_points(points.begin(), points.end());
//     thrust::device_vector<int> d_bin_indices(num_points, 0);
//     thrust::device_vector<float> d_bin_avgs(NUM_BINS, 0.0f);
//     thrust::device_vector<int> d_bin_counts(NUM_BINS, 0);
//     thrust::device_vector<int> d_labels(num_points, 0);

//     // Step 1: Bin points by radius
//     dim3 block(256);
//     dim3 grid((num_points + block.x - 1) / block.x);
//     binPoints<<<grid, block>>>(
//         thrust::raw_pointer_cast(d_points.data()),
//         thrust::raw_pointer_cast(d_bin_indices.data()),
//         num_points, bin_size);

//     // Step 2: Compute average intensity per bin
//     computeBinAverages<<<grid, block>>>(
//         thrust::raw_pointer_cast(d_points.data()),
//         thrust::raw_pointer_cast(d_bin_indices.data()),
//         thrust::raw_pointer_cast(d_bin_avgs.data()),
//         thrust::raw_pointer_cast(d_bin_counts.data()),
//         num_points);

//     // Normalize bin averages
//     thrust::device_vector<float> d_normalized_avgs(NUM_BINS, 0.0f);
//     thrust::transform(
//         d_bin_avgs.begin(), d_bin_avgs.end(),
//         d_bin_counts.begin(),
//         d_normalized_avgs.begin(),
//         [] __host__ __device__(float sum, int count) { return count > 0 ? sum / count : 0.0f; });

//     // Step 3: Classify points within bins
//     classifyPoints<<<grid, block>>>(
//         thrust::raw_pointer_cast(d_points.data()),
//         thrust::raw_pointer_cast(d_bin_indices.data()),
//         thrust::raw_pointer_cast(d_normalized_avgs.data()),
//         thrust::raw_pointer_cast(d_labels.data()),
//         num_points);

//     // Retrieve results
//     std::vector<int> h_labels(d_labels.begin(), d_labels.end());
//     std::vector<Point> h_points(d_points.begin(), d_points.end());

//     // Assign colors based on labels
//     std::vector<std::pair<Point, std::string>> colored_points;
//     for (int i = 0; i < num_points; ++i) {
//         std::string color = h_labels[i] == 0 ? "Blue" : "Yellow";
//         colored_points.emplace_back(h_points[i], color);
//     }

//     return colored_points;
// }