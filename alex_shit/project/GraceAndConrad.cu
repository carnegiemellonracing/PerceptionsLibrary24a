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

struct CalculateAngles {
    __host__ __device__ float operator()(const float2 &point) const {
        float angle = atan2f(point.y, point.x);
        return angle < 0 ? angle + 2 * M_PI : angle;
    }
};

struct CalculateRanges {
    __host__ __device__ float operator()(const float2 &point) const {
        return sqrtf(point.x * point.x + point.y * point.y);
    }
};

struct FilterPoints {
    float slope, intercept, height_threshold;

    FilterPoints(float s, float i, float ht) : slope(s), intercept(i), height_threshold(ht) {}

    __host__ __device__ bool operator()(const float3 &point) const {
        float range = sqrtf(point.x * point.x + point.y * point.y);
        return point.z > slope * range + intercept + height_threshold;
    }
};

thrust::host_vector<float3> GraceAndConrad(
    const thrust::host_vector<float3> &points,
    float alpha,
    int num_bins,
    float height_threshold) {

    thrust::device_vector<float3> d_points = points;
    thrust::device_vector<float2> d_xy(points.size());

    thrust::transform(d_points.begin(), d_points.end(), d_xy.begin(), [] __host__ __device__(const float3 &p) {
        return float2{p.x, p.y};
    });

    thrust::device_vector<float> d_angles(points.size());
    thrust::transform(d_xy.begin(), d_xy.end(), d_angles.begin(), CalculateAngles());

    thrust::device_vector<float> d_ranges(points.size());
    thrust::transform(d_xy.begin(), d_xy.end(), d_ranges.begin(), CalculateRanges());

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

    thrust::device_vector<int> d_grid_cell_indices(points.size());
    thrust::transform(
        d_segments.begin(), d_segments.end(), d_regments.begin(),
        d_grid_cell_indices.begin(),
        thrust::placeholders::_1 * num_bins + thrust::placeholders::_2);

    thrust::host_vector<float3> result;
    for (int seg_idx = 0; seg_idx < num_segments; ++seg_idx) {
        thrust::device_vector<float3> d_segment_points(points.size());
        auto end_it = thrust::copy_if(
            d_points.begin(), d_points.end(), d_segments.begin(),
            d_segment_points.begin(),
            [=] __host__ __device__(int segment) { return segment == seg_idx; });

        d_segment_points.resize(end_it - d_segment_points.begin());
        if (d_segment_points.empty()) continue;

        thrust::device_vector<float> X(d_segment_points.size());
        thrust::device_vector<float> Y(d_segment_points.size());

        thrust::transform(d_segment_points.begin(), d_segment_points.end(), X.begin(), [] __host__ __device__(const float3 &p) {
            return sqrtf(p.x * p.x + p.y * p.y);
        });

        thrust::transform(d_segment_points.begin(), d_segment_points.end(), Y.begin(), [] __host__ __device__(const float3 &p) {
            return p.z;
        });

        float x_bar = thrust::reduce(X.begin(), X.end()) / X.size();
        float y_bar = thrust::reduce(Y.begin(), Y.end()) / Y.size();

        thrust::device_vector<float> x_dev(X.size());
        thrust::transform(X.begin(), X.end(), thrust::constant_iterator<float>(x_bar), x_dev.begin(), thrust::minus<float>());
        thrust::device_vector<float> y_dev(Y.size());
        thrust::transform(Y.begin(), Y.end(), thrust::constant_iterator<float>(y_bar), y_dev.begin(), thrust::minus<float>());

        float numerator = thrust::inner_product(x_dev.begin(), x_dev.end(), y_dev.begin(), 0.0f);
        float denominator = thrust::inner_product(x_dev.begin(), x_dev.end(), x_dev.begin(), 0.0f);
        float slope = denominator != 0 ? numerator / denominator : 0;
        float intercept = y_bar - slope * x_bar;

        thrust::device_vector<float3> d_filtered_points(points.size());
        auto filtered_end = thrust::copy_if(
            d_segment_points.begin(), d_segment_points.end(),
            d_filtered_points.begin(),
            FilterPoints(slope, intercept, height_threshold));

        d_filtered_points.resize(filtered_end - d_filtered_points.begin());
        result.insert(result.end(), d_filtered_points.begin(), d_filtered_points.end());
    }

    return result;
}