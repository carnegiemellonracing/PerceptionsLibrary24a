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





#include <cuda_runtime.h>
#include <iostream>
#include <vector>
#include <cmath>

#define MAX_DISTANCE 30.0f // Maximum allowable distance
#define ERROR_MARGIN 0.15f  // Outlier threshold

struct Point {
    float x, y, z;

    __host__ __device__
    Point(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}
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

    float range = sqrtf(p.x * p.x + p.y * p.y);
    if (range > MAX_DISTANCE) return; // Discard points further than MAX_DISTANCE

    segments[idx] = min(static_cast<int>((angle - angle_min) / (angle_max - angle_min) * num_segments), num_segments - 1);
    bins[idx] = min(static_cast<int>((range - range_min) / (range_max - range_min) * num_bins), num_bins - 1);
}




__global__ void fitPlaneInGrid(
    const Point* points, const int* segments, const int* bins,
    float* plane_params, int num_points, int num_segments, int num_bins) {

    int segment = blockIdx.x;
    int bin = threadIdx.x;

    int cell_idx = segment * num_bins + bin;

    // Accumulators for plane fitting
    float sum_x = 0, sum_y = 0, sum_z = 0;
    float sum_x2 = 0, sum_y2 = 0, sum_xy = 0;
    float sum_xz = 0, sum_yz = 0;
    int count = 0;

    for (int i = 0; i < num_points; ++i) {
        if (segments[i] == segment && bins[i] == bin) {
            Point p = points[i];
            sum_x += p.x;
            sum_y += p.y;
            sum_z += p.z;
            sum_x2 += p.x * p.x;
            sum_y2 += p.y * p.y;
            sum_xy += p.x * p.y;
            sum_xz += p.x * p.z;
            sum_yz += p.y * p.z;
            count++;
        }
    }

    if (count > 2) {
        float det = sum_x2 * sum_y2 - sum_xy * sum_xy;
        if (fabsf(det) > 1e-6) {
            plane_params[cell_idx * 3 + 0] = (sum_y2 * sum_xz - sum_xy * sum_yz) / det; // a
            plane_params[cell_idx * 3 + 1] = (sum_x2 * sum_yz - sum_xy * sum_xz) / det; // b
            plane_params[cell_idx * 3 + 2] = (sum_z - plane_params[cell_idx * 3 + 0] * sum_x - plane_params[cell_idx * 3 + 1] * sum_y) / count; // c
        } else {
            plane_params[cell_idx * 3 + 0] = 0;
            plane_params[cell_idx * 3 + 1] = 0;
            plane_params[cell_idx * 3 + 2] = 0;
        }
    } else {
        plane_params[cell_idx * 3 + 0] = 0;
        plane_params[cell_idx * 3 + 1] = 0;
        plane_params[cell_idx * 3 + 2] = 0;
    }
}


__global__ void validatePlaneOutliers(
    const Point* points, const int* segments, const int* bins,
    const float* plane_params, Point* outliers, int* outlier_count,
    float margin_of_error, int num_points, int num_segments, int num_bins) {

    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_points) return;

    int segment = segments[idx];
    int bin = bins[idx];
    if (segment >= num_segments || bin >= num_bins) return;

    int cell_idx = segment * num_bins + bin;
    float a = plane_params[cell_idx * 3 + 0];
    float b = plane_params[cell_idx * 3 + 1];
    float c = plane_params[cell_idx * 3 + 2];

    Point p = points[idx];
    float predicted_z = a * p.x + b * p.y + c;

    // Not absolute, want it to be above ground plane
    if ((p.z - predicted_z) > margin_of_error) {
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

    float* d_plane_params;
    cudaMalloc(&d_plane_params, num_segments * num_bins * 3 * sizeof(float));

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
    fitPlaneInGrid<<<segmentGrid, binGrid>>>(d_points, d_segments, d_bins,
                                             d_plane_params, num_points, num_segments, num_bins);

    validatePlaneOutliers<<<grid, block>>>(d_points, d_segments, d_bins,
                                           d_plane_params, d_outliers, d_outlier_count,
                                           ERROR_MARGIN, num_points, num_segments, num_bins);

    // Copy results back to host
    int h_outlier_count;
    cudaMemcpy(&h_outlier_count, d_outlier_count, sizeof(int), cudaMemcpyDeviceToHost);

    std::vector<Point> h_outliers(h_outlier_count);
    cudaMemcpy(h_outliers.data(), d_outliers, h_outlier_count * sizeof(Point), cudaMemcpyDeviceToHost);

    // Free device memory
    cudaFree(d_points);
    cudaFree(d_segments);
    cudaFree(d_bins);
    cudaFree(d_plane_params);
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
        std::vector<Point> std_points(points.begin(), points.end());


        int num_points = points.size();


        thrust::device_vector<Point> d_points = points;


        thrust::device_vector<float> d_angles(num_points);
        thrust::transform(d_points.begin(), d_points.end(), d_angles.begin(),
                        [] __host__ __device__(const Point& p) {
                            float angle = atan2f(p.y, p.x);
                            return (angle < 0) ? angle + 2 * M_PI : angle;
                        });
        
        
        float angle_min = *thrust::min_element(d_angles.begin(), d_angles.end());
        float angle_max = *thrust::max_element(d_angles.begin(), d_angles.end());
        int num_segments = static_cast<int>((angle_max - angle_min) / alpha) + 1;


        cudaEvent_t start, stop;
        cudaEventCreate(&start);
        cudaEventCreate(&stop);

        cudaEventRecord(start);
        auto result = processPointsCUDA(std_points, num_segments, num_bins);
        cudaEventRecord(stop);

        cudaEventSynchronize(stop);

        float milliseconds = 0;
        cudaEventElapsedTime(&milliseconds, start, stop);
        std::cout << "processPointsCUDA execution time: " << milliseconds << " ms" << std::endl;

        // Clean up CUDA events
        cudaEventDestroy(start);
        cudaEventDestroy(stop);




        std::cout << "GraceAndConrad completed successfully." << std::endl;
        std::cout << "Number of points returned: " << result.size() << std::endl;

        // Convert std::vector<Point> back to thrust::host_vector<Point> for returning
        return thrust::host_vector<Point>(result.begin(), result.end());
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