#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/transform.h>
#include <thrust/reduce.h>
#include <thrust/sequence.h>
#include <thrust/copy.h>
#include <thrust/sort.h>
#include <thrust/iterator/zip_iterator.h>
#include <thrust/functional.h>
#include <thrust/tuple.h>
#include <cmath>
#include <chrono>





#include <cuda_runtime.h>
#include <iostream>
#include <vector>


#define MAX_DISTANCE 40.0f // Maximum allowable distance
#define ERROR_MARGIN 0.15f  // Outlier threshold
#define MAX_HEIGHT 0.4f

struct Point {
    float x, y, z, intensity;

    __host__ __device__
    Point(float x = 0, float y = 0, float z = 0, float intensity = 0) : x(x), y(y), z(z), intensity(intensity) {}
};




__global__ void assignToGrid(
    const Point* points, int* segments, int* bins, int num_points,
    float angle_min, float angle_max, float range_min, float range_max,
    int num_segments, int num_bins) {

    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_points) return;

    Point p = points[idx];
    float angle = atan2f(p.y, p.x);
    if (angle < 0) angle += 2 * M_PI;

    float range = sqrtf((p.x * p.x) + (p.y * p.y));

    if (fabsf(p.x) > MAX_DISTANCE || fabsf(p.y) > MAX_DISTANCE) return;

    if (range > MAX_DISTANCE) return; // Discard points further than MAX_DISTANCE


    segments[idx] = min(static_cast<int>((angle - angle_min) / (angle_max - angle_min) * num_segments), num_segments - 1);
    bins[idx] = min(static_cast<int>((range - range_min) / (range_max - range_min) * num_bins), num_bins - 1);
}




__global__ void findLowestPointInBin(
    const Point* points, const int* segments, const int* bins, Point* bin_lowest_points,
    int num_points, int num_segments, int num_bins) {

    int segment = blockIdx.x;
    int bin = threadIdx.x;

    int cell_idx = segment * num_bins + bin;
    float min_z = FLT_MAX;
    Point lowest_point;

    for (int i = 0; i < num_points; i++) {
        if (segments[i] == segment && bins[i] == bin) {
            if (points[i].z < min_z) {
                min_z = points[i].z;
                lowest_point = points[i];
            }
        }
    }

    if (min_z < FLT_MAX) {
        bin_lowest_points[cell_idx] = lowest_point;
    } else {
        bin_lowest_points[cell_idx] = Point(0, 0, FLT_MAX); // No valid points in bin
    }
}

__global__ void fitLineInSegment(
    const Point* bin_lowest_points, float* segment_line_params, int num_segments, int num_bins) {

    int segment = blockIdx.x;

    float x_sum = 0, z_sum = 0, x2_sum = 0, xz_sum = 0;
    int count = 0;

    for (int bin = 0; bin < num_bins; bin++) {
        int idx = segment * num_bins + bin;
        Point p = bin_lowest_points[idx];
        if (p.z != FLT_MAX) { // Ignore empty bins
            float range = sqrtf(p.x * p.x + p.y * p.y);
            x_sum += range;
            z_sum += p.z;
            x2_sum += range * range;
            xz_sum += range * p.z;
            count++;
        }
    }

    if (count > 1) {
        float slope = (count * xz_sum - x_sum * z_sum) / (count * x2_sum - x_sum * x_sum);
        float intercept = (z_sum - slope * x_sum) / count;
        segment_line_params[segment * 2 + 0] = slope;
        segment_line_params[segment * 2 + 1] = intercept;
    } else {
        segment_line_params[segment * 2 + 0] = 0; // Default flat line
        segment_line_params[segment * 2 + 1] = 0;
    }
}

__global__ void validateOutliers(
    const Point* points, const int* segments, const float* segment_line_params,
    Point* outliers, int* outlier_count, float margin_of_error, int num_points, int num_segments) {

    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_points) return;

    int segment = segments[idx];
    if (segment >= num_segments) return;

    float slope = segment_line_params[segment * 2 + 0];
    float intercept = segment_line_params[segment * 2 + 1];

    Point p = points[idx];
    float range = sqrtf(p.x * p.x + p.y * p.y);
    float predicted_z = slope * range + intercept;

    if (sqrtf((p.x * p.x) + (p.y * p.y)) > MAX_DISTANCE) return;

    if ((p.z - predicted_z) > margin_of_error && (p.z - predicted_z) < MAX_HEIGHT) {
        int out_idx = atomicAdd(outlier_count, 1);
        outliers[out_idx] = p;
    }
}

std::vector<Point> processPointsCUDA(
    const std::vector<Point>& h_points, int num_segments, int num_bins) {

    int num_points = h_points.size();

    // Allocate device memory
    Point* d_points;
    cudaMalloc(&d_points, num_points * sizeof(Point));
    cudaMemcpy(d_points, h_points.data(), num_points * sizeof(Point), cudaMemcpyHostToDevice);

    int* d_segments;
    int* d_bins;
    cudaMalloc(&d_segments, num_points * sizeof(int));
    cudaMalloc(&d_bins, num_points * sizeof(int));

    Point* d_bin_lowest_points;
    cudaMalloc(&d_bin_lowest_points, num_segments * num_bins * sizeof(Point));

    float* d_segment_line_params;
    cudaMalloc(&d_segment_line_params, num_segments * 2 * sizeof(float));

    Point* d_outliers;
    int* d_outlier_count;
    cudaMalloc(&d_outliers, num_points * sizeof(Point));
    cudaMalloc(&d_outlier_count, sizeof(int));
    cudaMemset(d_outlier_count, 0, sizeof(int));

    // Launch kernels
    dim3 block(256);
    dim3 grid((num_points + block.x - 1) / block.x);
    assignToGrid<<<grid, block>>>(d_points, d_segments, d_bins, num_points,
                                  -M_PI, M_PI, 0.0f, MAX_DISTANCE, num_segments, num_bins);

    dim3 segmentGrid(num_segments);
    dim3 binGrid(num_bins);
    findLowestPointInBin<<<segmentGrid, binGrid>>>(d_points, d_segments, d_bins,
                                                   d_bin_lowest_points, num_points, num_segments, num_bins);

    fitLineInSegment<<<segmentGrid, 1>>>(d_bin_lowest_points, d_segment_line_params, num_segments, num_bins);

    validateOutliers<<<grid, block>>>(d_points, d_segments, d_segment_line_params,
                                      d_outliers, d_outlier_count, ERROR_MARGIN, num_points, num_segments);

    // Copy results back to host
    int h_outlier_count;
    cudaMemcpy(&h_outlier_count, d_outlier_count, sizeof(int), cudaMemcpyDeviceToHost);

    std::vector<Point> h_outliers(h_outlier_count);
    cudaMemcpy(h_outliers.data(), d_outliers, h_outlier_count * sizeof(Point), cudaMemcpyDeviceToHost);

    // Free device memory
    cudaFree(d_points);
    cudaFree(d_segments);
    cudaFree(d_bins);
    cudaFree(d_bin_lowest_points);
    cudaFree(d_segment_line_params);
    cudaFree(d_outliers);
    cudaFree(d_outlier_count);

    return h_outliers;
}




thrust::host_vector<Point> GraceAndConrad(
    const thrust::host_vector<Point>& points,
    float alpha,
    int num_bins,
    float height_threshold) {
    try {

        // Convert thrust::host_vector to std::vector
        auto start = std::chrono::high_resolution_clock::now();
        std::vector<Point> std_points(points.begin(), points.end());
        auto end = std::chrono::high_resolution_clock::now();
        std::cout << "Convert thrust::host_vector to std::vector: "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms\n";



        // Count points
        start = std::chrono::high_resolution_clock::now();
        int num_points = points.size();
        end = std::chrono::high_resolution_clock::now();
        std::cout << "Count points: "
                  << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << " µs\n";

        // Copy points to device
        start = std::chrono::high_resolution_clock::now();
        thrust::device_vector<Point> d_points = points;
        end = std::chrono::high_resolution_clock::now();
        std::cout << "Copy points to device: "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms\n";

        // Compute angles
        start = std::chrono::high_resolution_clock::now();
        thrust::device_vector<float> d_angles(num_points);
        thrust::transform(d_points.begin(), d_points.end(), d_angles.begin(),
                          [] __host__ __device__(const Point& p) {
                              float angle = atan2f(p.y, p.x);
                              return (angle < 0) ? angle + 2 * M_PI : angle;
                          });
        end = std::chrono::high_resolution_clock::now();
        std::cout << "Compute angles: "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms\n";

        // Compute min/max angles and number of segments
        start = std::chrono::high_resolution_clock::now();
        float angle_min = *thrust::min_element(d_angles.begin(), d_angles.end());
        float angle_max = *thrust::max_element(d_angles.begin(), d_angles.end());
        int num_segments = static_cast<int>((angle_max - angle_min) / alpha) + 1;
        end = std::chrono::high_resolution_clock::now();
        std::cout << "Compute min/max angles and segments: "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms\n";

        // Process points in CUDA
        cudaEvent_t start_event, stop_event;
        cudaEventCreate(&start_event);
        cudaEventCreate(&stop_event);

        cudaEventRecord(start_event);
        auto result = processPointsCUDA(std_points, num_segments, num_bins);
        cudaEventRecord(stop_event);

        cudaEventSynchronize(stop_event);

        float cuda_time = 0;
        cudaEventElapsedTime(&cuda_time, start_event, stop_event);
        std::cout << "processPointsCUDA execution time: " << cuda_time << " ms\n";

        // Clean up CUDA events
        cudaEventDestroy(start_event);
        cudaEventDestroy(stop_event);

        // Post-process result
        start = std::chrono::high_resolution_clock::now();
        thrust::host_vector<Point> result_vector(result.begin(), result.end());
        end = std::chrono::high_resolution_clock::now();
        std::cout << "Convert result to thrust::host_vector: "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms\n";

        std::cout << "GraceAndConrad completed successfully.\n";
        std::cout << "Number of points returned: " << result.size() << std::endl;

        return result_vector;
    } catch (const thrust::system_error& e) {
        std::cerr << "Thrust system error: " << e.what() << std::endl;
        return thrust::host_vector<Point>();
    } catch (const std::exception& e) {
        std::cerr << "Standard exception: " << e.what() << std::endl;
        return thrust::host_vector<Point>();
    } catch (...) {
        std::cerr << "Unknown error occurred in GraceAndConrad." << std::endl;
        return thrust::host_vector<Point>();
    }
}

