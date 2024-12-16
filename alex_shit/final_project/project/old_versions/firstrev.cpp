#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/transform.h>
#include <thrust/sequence.h>
#include <thrust/functional.h>
#include <thrust/sort.h>
#include <thrust/reduce.h>
#include <thrust/copy.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <iostream>

// Functor to calculate angles for each point
struct CalculateAngles {
    __host__ __device__ float operator()(const float2 &point) const {
        float angle = atan2f(point.y, point.x);
        return angle < 0 ? angle + 2 * M_PI : angle;
    }
};

// Functor to calculate range for each point
struct CalculateRanges {
    __host__ __device__ float operator()(const float2 &point) const {
        return sqrtf(point.x * point.x + point.y * point.y);
    }
};

// Functor to filter points based on linear regression and height threshold
struct FilterPoints {
    float slope, intercept, height_threshold;

    FilterPoints(float s, float i, float ht) : slope(s), intercept(i), height_threshold(ht) {}

    __host__ __device__ bool operator()(const float3 &point) const {
        float range = sqrtf(point.x * point.x + point.y * point.y);
        return point.z > slope * range + intercept + height_threshold;
    }
};

// Linear regression functor
struct LinearRegression {
    float slope, intercept;

    LinearRegression(const thrust::device_vector<float> &X, const thrust::device_vector<float> &Y) {
        float x_bar = thrust::reduce(X.begin(), X.end()) / X.size();
        float y_bar = thrust::reduce(Y.begin(), Y.end()) / Y.size();

        thrust::device_vector<float> x_dev(X.size());
        thrust::transform(X.begin(), X.end(), thrust::constant_iterator<float>(x_bar), x_dev.begin(), thrust::minus<float>());
        thrust::device_vector<float> y_dev(Y.size());
        thrust::transform(Y.begin(), Y.end(), thrust::constant_iterator<float>(y_bar), y_dev.begin(), thrust::minus<float>());

        float numerator = thrust::inner_product(x_dev.begin(), x_dev.end(), y_dev.begin(), 0.0f);
        float denominator = thrust::inner_product(x_dev.begin(), x_dev.end(), x_dev.begin(), 0.0f);
        slope = denominator != 0 ? numerator / denominator : 0;
        intercept = y_bar - slope * x_bar;
    }
};

// Main GraceAndConrad implementation
thrust::host_vector<float3> GraceAndConrad(
    const thrust::host_vector<float3> &points,
    float alpha,
    int num_bins,
    float height_threshold) {

    // Transfer input points to device
    thrust::device_vector<float3> d_points = points;
    thrust::device_vector<float2> d_xy(points.size());

    // Extract x, y coordinates
    thrust::transform(d_points.begin(), d_points.end(), d_xy.begin(), [] __host__ __device__(const float3 &p) {
        return float2{p.x, p.y};
    });

    // Calculate angles and ranges
    thrust::device_vector<float> d_angles(points.size());
    thrust::transform(d_xy.begin(), d_xy.end(), d_angles.begin(), CalculateAngles());

    thrust::device_vector<float> d_ranges(points.size());
    thrust::transform(d_xy.begin(), d_xy.end(), d_ranges.begin(), CalculateRanges());

    // Determine angle bins
    float angle_min = *thrust::min_element(d_angles.begin(), d_angles.end());
    float angle_max = *thrust::max_element(d_angles.begin(), d_angles.end());
    int num_segments = static_cast<int>((angle_max - angle_min) / alpha) + 1;
    thrust::device_vector<float> d_angle_bins(num_segments);
    thrust::sequence(d_angle_bins.begin(), d_angle_bins.end(), angle_min, alpha);

    thrust::device_vector<int> d_segments(points.size());
    thrust::lower_bound(
        d_angle_bins.begin(), d_angle_bins.end(),
        d_angles.begin(), d_angles.end(),
        d_segments.begin());

    // Determine range bins
    float range_min = *thrust::min_element(d_ranges.begin(), d_ranges.end());
    float range_max = *thrust::max_element(d_ranges.begin(), d_ranges.end());
    float bin_size = (range_max - range_min) / num_bins;
    thrust::device_vector<float> d_range_bins(num_bins);
    thrust::sequence(d_range_bins.begin(), d_range_bins.end(), range_min, bin_size);

    thrust::device_vector<int> d_regments(points.size());
    thrust::lower_bound(
        d_range_bins.begin(), d_range_bins.end(),
        d_ranges.begin(), d_ranges.end(),
        d_regments.begin());

    // Compute grid cell indices
    thrust::device_vector<int> d_grid_cell_indices(points.size());
    thrust::transform(
        d_segments.begin(), d_segments.end(), d_regments.begin(),
        d_grid_cell_indices.begin(),
        thrust::placeholders::_1 * num_bins + thrust::placeholders::_2);

    // Iterate over segments
    thrust::host_vector<float3> result;
    for (int seg_idx = 0; seg_idx < num_segments; ++seg_idx) {
        // Filter points belonging to this segment
        thrust::device_vector<float3> d_segment_points(points.size());
        auto end_it = thrust::copy_if(
            d_points.begin(), d_points.end(), d_segments.begin(),
            d_segment_points.begin(),
            [=] __host__ __device__(int segment) { return segment == seg_idx; });

        d_segment_points.resize(end_it - d_segment_points.begin());

        // Skip empty segments
        if (d_segment_points.empty()) continue;

        // Perform bin filtering and linear regression
        thrust::device_vector<float> X(d_segment_points.size());
        thrust::device_vector<float> Y(d_segment_points.size());

        thrust::transform(d_segment_points.begin(), d_segment_points.end(), X.begin(), [] __host__ __device__(const float3 &p) {
            return sqrtf(p.x * p.x + p.y * p.y);
        });

        thrust::transform(d_segment_points.begin(), d_segment_points.end(), Y.begin(), [] __host__ __device__(const float3 &p) {
            return p.z;
        });

        LinearRegression lr(X, Y);

        // Filter points based on regression
        thrust::device_vector<float3> d_filtered_points(points.size());
        auto filtered_end = thrust::copy_if(
            d_segment_points.begin(), d_segment_points.end(),
            d_filtered_points.begin(),
            FilterPoints(lr.slope, lr.intercept, height_threshold));

        d_filtered_points.resize(filtered_end - d_filtered_points.begin());

        // Append to results
        result.insert(result.end(), d_filtered_points.begin(), d_filtered_points.end());
    }

    return result;
}



int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <csv_file_path>" << std::endl;
        return 1;
    }

    std::string csv_file_path = argv[1];
    thrust::host_vector<float3> points;

    try {
        // Load points from CSV file
        csv::CSVReader reader(csv_file_path);
        for (csv::CSVRow &row : reader) {
            float x = row["x"].get<float>();
            float y = row["y"].get<float>();
            float z = row["z"].get<float>();
            points.push_back(make_float3(x, y, z));
        }
    } catch (const std::exception &e) {
        std::cerr << "Error reading CSV file: " << e.what() << std::endl;
        return 1;
    }

    // Parameters for GraceAndConrad
    float alpha = 0.1f;
    int num_bins = 10;
    float height_threshold = 0.5f;

    // Call GraceAndConrad
    auto result = GraceAndConrad(points, alpha, num_bins, height_threshold);

    // Output results
    std::cout << "Filtered points:" << std::endl;
    for (const auto &p : result) {
        std::cout << p.x << "," << p.y << "," << p.z << std::endl;
    }

    return 0;
}