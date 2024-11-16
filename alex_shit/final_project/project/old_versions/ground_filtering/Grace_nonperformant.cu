// Updated

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

// Structure for points
struct Point {
    float x, y, z;

    __host__ __device__
    Point(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}
};

// Kernel to assign points to grid cells
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
    segments[idx] = min(static_cast<int>((angle - angle_min) / (angle_max - angle_min) * num_segments), num_segments - 1);
    bins[idx] = min(static_cast<int>((range - range_min) / (range_max - range_min) * num_bins), num_bins - 1);
}

// Kernel to find the lowest Z-value in each grid cell
__global__ void findLowestZ(
    const Point* points, const int* segments, const int* bins,
    Point* lowest_points, bool* has_point, int num_points, int num_segments, int num_bins) {

    int segment = blockIdx.x;
    int bin = threadIdx.x;

    float min_z = INFINITY;
    Point lowest_point;
    bool found = false;

    for (int i = 0; i < num_points; i++) {
        if (segments[i] == segment && bins[i] == bin) {
            if (points[i].z < min_z) {
                min_z = points[i].z;
                lowest_point = points[i];
                found = true;
            }
        }
    }

    int cell_idx = segment * num_bins + bin;
    if (found) {
        lowest_points[cell_idx] = lowest_point;
        has_point[cell_idx] = true;
    } else {
        has_point[cell_idx] = false;
    }
}

// Kernel to perform regression for each segment
__global__ void calculateRegression(
    const Point* lowest_points, const bool* has_point,
    float* slopes, float* intercepts, int num_segments, int num_bins) {

    int segment = blockIdx.x;

    float x_sum = 0, y_sum = 0, x2_sum = 0, xy_sum = 0;
    int count = 0;

    for (int bin = 0; bin < num_bins; bin++) {
        int idx = segment * num_bins + bin;
        if (has_point[idx]) {
            Point p = lowest_points[idx];
            float x = sqrtf(p.x * p.x + p.y * p.y); // Range
            float y = p.z; // Height
            x_sum += x;
            y_sum += y;
            x2_sum += x * x;
            xy_sum += x * y;
            count++;
        }
    }

    if (count > 1) {
        slopes[segment] = (count * xy_sum - x_sum * y_sum) / (count * x2_sum - x_sum * x_sum);
        intercepts[segment] = (y_sum - slopes[segment] * x_sum) / count;
    } else {
        slopes[segment] = 0;
        intercepts[segment] = 0;
    }
}

// Kernel to validate points against trendlines
__global__ void validateAgainstTrendline(
    const Point* points, const int* segments, const int* bins,
    const float* slopes, const float* intercepts,
    Point* outliers, int* outlier_count, float margin_of_error,
    int num_points, int num_segments) {

    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_points) return;

    int segment = segments[idx];
    if (segment >= num_segments) return;

    float slope = slopes[segment];
    float intercept = intercepts[segment];

    float range = sqrtf(points[idx].x * points[idx].x + points[idx].y * points[idx].y);
    float predicted_z = slope * range + intercept;

    if (fabsf(points[idx].z - predicted_z) > margin_of_error) {
        int out_idx = atomicAdd(outlier_count, 1);
        outliers[out_idx] = points[idx];
    }
}




// Host function for executing the CUDA implementation
std::vector<Point> processPointsCUDA(
    const std::vector<Point>& h_points, float angle_min, float angle_max,
    float range_min, float range_max, int num_segments, int num_bins, float margin_of_error) {

    int num_points = h_points.size();

    // Allocate memory for points and grid
    Point* d_points;
    cudaMalloc(&d_points, num_points * sizeof(Point));
    cudaMemcpy(d_points, h_points.data(), num_points * sizeof(Point), cudaMemcpyHostToDevice);

    int* d_segments;
    int* d_bins;
    cudaMalloc(&d_segments, num_points * sizeof(int));
    cudaMalloc(&d_bins, num_points * sizeof(int));

    Point* d_lowest_points;
    bool* d_has_point;
    cudaMalloc(&d_lowest_points, num_segments * num_bins * sizeof(Point));
    cudaMalloc(&d_has_point, num_segments * num_bins * sizeof(bool));

    float* d_slopes;
    float* d_intercepts;
    cudaMalloc(&d_slopes, num_segments * sizeof(float));
    cudaMalloc(&d_intercepts, num_segments * sizeof(float));

    Point* d_outliers;
    int* d_outlier_count;
    cudaMalloc(&d_outliers, num_points * sizeof(Point));
    cudaMalloc(&d_outlier_count, sizeof(int));
    cudaMemset(d_outlier_count, 0, sizeof(int));

    // Launch kernels
    dim3 block(256);
    dim3 grid((num_points + block.x - 1) / block.x);
    assignToGrid<<<grid, block>>>(d_points, d_segments, d_bins, num_points,
                                  angle_min, angle_max, range_min, range_max, num_segments, num_bins);

    dim3 segmentGrid(num_segments);
    dim3 binGrid(num_bins);
    findLowestZ<<<segmentGrid, binGrid>>>(d_points, d_segments, d_bins,
                                          d_lowest_points, d_has_point, num_points, num_segments, num_bins);

    calculateRegression<<<segmentGrid, 1>>>(d_lowest_points, d_has_point,
                                            d_slopes, d_intercepts, num_segments, num_bins);

    validateAgainstTrendline<<<grid, block>>>(d_points, d_segments, d_bins,
                                              d_slopes, d_intercepts,
                                              d_outliers, d_outlier_count,
                                              margin_of_error, num_points, num_segments);

    // Copy results back to host
    int h_outlier_count;
    cudaMemcpy(&h_outlier_count, d_outlier_count, sizeof(int), cudaMemcpyDeviceToHost);

    std::vector<Point> h_outliers(h_outlier_count);
    cudaMemcpy(h_outliers.data(), d_outliers, h_outlier_count * sizeof(Point), cudaMemcpyDeviceToHost);

    // Free device memory
    cudaFree(d_points);
    cudaFree(d_segments);
    cudaFree(d_bins);
    cudaFree(d_lowest_points);
    cudaFree(d_has_point);
    cudaFree(d_slopes);
    cudaFree(d_intercepts);
    cudaFree(d_outliers);
    cudaFree(d_outlier_count);

    return h_outliers;
}







thrust::host_vector<Point> GraceAndConrad(
    const thrust::host_vector<Point>& points,
    float alpha,
    int num_bins,
    float height_threshold) {


    cudaEvent_t start, stop;
    float milliseconds = 0;

    // Create events
    cudaEventCreate(&start);
    cudaEventCreate(&stop);

    // Start timing
    cudaEventRecord(start);


    int num_points = points.size();
    thrust::device_vector<Point> d_points = points;

    // Debug: Check points size
    std::cout << "Number of points: " << num_points << std::endl;

    thrust::device_vector<float> d_angles(num_points);
    thrust::transform(d_points.begin(), d_points.end(), d_angles.begin(),
                      [] __host__ __device__(const Point& p) {
                          float angle = atan2f(p.y, p.x);
                          return (angle < 0) ? angle + 2 * M_PI : angle;
                      });

    thrust::device_vector<float> d_ranges(num_points);
    thrust::transform(d_points.begin(), d_points.end(), d_ranges.begin(),
                      [] __host__ __device__(const Point& p) {
                          return sqrtf(p.x * p.x + p.y * p.y);
                      });

    float angle_min = *thrust::min_element(d_angles.begin(), d_angles.end());
    float angle_max = *thrust::max_element(d_angles.begin(), d_angles.end());
    int num_segments = static_cast<int>((angle_max - angle_min) / alpha) + 1;

    std::cout << "Number of segments: " << num_segments << std::endl;

    thrust::device_vector<int> d_segments(num_points);
    thrust::transform(d_angles.begin(), d_angles.end(), d_segments.begin(),
                      [=] __host__ __device__(float angle) {
                          return static_cast<int>((angle - angle_min) / alpha);
                      });

    float range_min = *thrust::min_element(d_ranges.begin(), d_ranges.end());
    float range_max = *thrust::max_element(d_ranges.begin(), d_ranges.end());
    float bin_size = (range_max - range_min) / num_bins;


    // // Copy back to host to print
    // thrust::host_vector<int> h_segments = d_segments;

    // // Print the segments
    // std::cout << "Segments: ";
    // for (int i = 0; i < h_segments.size(); ++i) {
    //     std::cout << h_segments[i] << " ";
    // }
    // std::cout << std::endl;

    std::cout << "Range min: " << range_min << ", Range max: " << range_max << ", Bin size: " << bin_size << std::endl;

    thrust::device_vector<int> d_regments(num_points);
    thrust::transform(d_ranges.begin(), d_ranges.end(), d_regments.begin(),
                      [=] __host__ __device__(float range) {
                          return static_cast<int>((range - range_min) / bin_size);
                      });

    
    // // Copy back to host to print
    // thrust::host_vector<int> h_regments = d_regments;
    
    // std::cout << "Regments: ";
    // for (int i = 0; i < h_regments.size(); ++i) {
    //     std::cout << h_regments[i] << " ";
    // }
    // std::cout << std::endl;

    // Prepare outputs
    thrust::device_vector<int> d_filtered_counts(num_segments, 0);
    thrust::device_vector<Point> d_filtered_points(num_segments * num_points);


    // Stop timing before kernel launch
    cudaEventRecord(stop);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&milliseconds, start, stop);

    std::cout << "Time elapsed up to kernel launch: " << milliseconds << " ms" << std::endl;

    // Clean up events
    cudaEventDestroy(start);
    cudaEventDestroy(stop);



    try {
        // Convert thrust::host_vector to std::vector
        std::vector<Point> std_points(points.begin(), points.end());


        cudaEvent_t start, stop;
        cudaEventCreate(&start);
        cudaEventCreate(&stop);

        cudaEventRecord(start);
        auto result = processPointsCUDA(std_points, angle_min, angle_max, range_min, range_max, num_segments, num_bins, height_threshold);
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