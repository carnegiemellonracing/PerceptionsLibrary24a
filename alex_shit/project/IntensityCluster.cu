#include <vector>
#include <cmath>
#include <string>
#include <algorithm>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include "Point.hpp"

// Global constants
const float EPS = 5.5f;       // Large `eps` for clustering
const int MIN_SAMPLES = 3;    // Minimum points for a valid cluster
const float TRACK_WIDTH = 3.0f; // Approximate track width for labeling

// CUDA kernel to compute pairwise distances for DBSCAN
__global__ void computePairwiseDistances(const Point* points, int num_points, float* distances) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    int j = blockIdx.y * blockDim.y + threadIdx.y;

    if (i < num_points && j < num_points && i != j) {
        float dx = points[i].x - points[j].x;
        float dy = points[i].y - points[j].y;
        float dist = sqrtf(dx * dx + dy * dy);
        distances[i * num_points + j] = dist;
    }
}


// Compute midline by fitting a spline (simplified as averaging)
std::vector<Point> computeMidline(const std::vector<Point>& points, const std::vector<int>& cluster) {
    std::vector<Point> midline;
    for (int idx : cluster) {
        midline.push_back(points[idx]);
    }
    return midline; // In practice, a cubic spline fit would be applied here
}

// Label cones as "blue" or "yellow"
std::vector<std::pair<Point, std::string>> labelCones(const std::vector<Point>& points, const std::vector<Point>& midline, const std::vector<int>& cluster) {
    std::vector<std::pair<Point, std::string>> labeled_cones;

    for (int idx : cluster) {
        const auto& point = points[idx];
        float min_dist = INFINITY;
        Point closest_midline_point;

        for (const auto& midline_point : midline) {
            float dx = point.x - midline_point.x;
            float dy = point.y - midline_point.y;
            float dist = sqrtf(dx * dx + dy * dy);

            if (dist < min_dist) {
                min_dist = dist;
                closest_midline_point = midline_point;
            }
        }

        if (min_dist <= TRACK_WIDTH / 2) {
            float dx = point.x - closest_midline_point.x;
            std::string label = (dx > 0) ? "blue" : "yellow";
            labeled_cones.emplace_back(point, label);
        }
    }

    return labeled_cones;
}


std::vector<std::vector<int>> NewRunDBSCAN(const std::vector<Point>& points, float eps, int min_samples) {
    int num_points = points.size();
    std::vector<std::vector<int>> clusters;

    // Create visited and labels arrays
    std::vector<bool> visited(num_points, false);
    std::vector<int> labels(num_points, -1);  // -1 means noise
    int cluster_id = 0;

    // Helper lambda to calculate distance
    auto distance = [](const Point& a, const Point& b) {
        float dx = a.x - b.x;
        float dy = a.y - b.y;
        float dz = a.z - b.z;
        return sqrtf(dx * dx + dy * dy + dz * dz);
    };

    // Helper function to find neighbors
    auto get_neighbors = [&](int idx) {
        std::vector<int> neighbors;
        for (int i = 0; i < num_points; ++i) {
            if (i != idx && distance(points[idx], points[i]) <= eps) {
                neighbors.push_back(i);
            }
        }
        return neighbors;
    };

    // Main DBSCAN logic
    for (int i = 0; i < num_points; ++i) {
        if (visited[i]) continue;
        visited[i] = true;

        // Find neighbors
        auto neighbors = get_neighbors(i);
        if (neighbors.size() < min_samples) {
            labels[i] = -1;  // Mark as noise
            continue;
        }

        // Create a new cluster
        clusters.emplace_back();
        labels[i] = cluster_id;
        clusters[cluster_id].push_back(i);

        // Expand the cluster
        std::vector<int> to_process = neighbors;
        while (!to_process.empty()) {
            int current = to_process.back();
            to_process.pop_back();

            if (!visited[current]) {
                visited[current] = true;

                // Get neighbors of the current point
                auto current_neighbors = get_neighbors(current);
                if (current_neighbors.size() >= min_samples) {
                    to_process.insert(to_process.end(), current_neighbors.begin(), current_neighbors.end());
                }
            }

            if (labels[current] == -1 || labels[current] == -2) {
                labels[current] = cluster_id;
                clusters[cluster_id].push_back(current);
            }
        }

        cluster_id++;
    }

    return clusters;
}



// Main pipeline
std::vector<std::pair<Point, std::string>> ClusterCones(const std::vector<Point>& points) {
    // Step 1: Run DBSCAN with large eps using the new implementation
    auto clusters = NewRunDBSCAN(points, EPS, MIN_SAMPLES);

    // Step 2: Find the cluster closest to the origin
    int best_cluster_idx = -1;
    float min_dist_to_origin = INFINITY;

    for (size_t i = 0; i < clusters.size(); ++i) {
        for (size_t j = 0; j < clusters[i].size(); ++j) {
            int idx = clusters[i][j];
            const Point& p = points[idx];
            float dist_to_origin = sqrtf(p.x * p.x + p.y * p.y);
            if (dist_to_origin < min_dist_to_origin) {
                min_dist_to_origin = dist_to_origin;
                best_cluster_idx = i;
            }
        }
    }

    if (best_cluster_idx == -1) {
        return {};
    }

    // Step 3: Compute midline
    std::vector<Point> midline;
    for (size_t i = 0; i < clusters[best_cluster_idx].size(); ++i) {
        int idx = clusters[best_cluster_idx][i];
        midline.push_back(points[idx]);
    }

    // Step 4: Label cones
    std::vector<std::pair<Point, std::string>> labeled_cones;
    for (size_t i = 0; i < clusters[best_cluster_idx].size(); ++i) {
        int idx = clusters[best_cluster_idx][i];
        const Point& point = points[idx];
        float min_dist = INFINITY;
        Point closest_midline_point;

        for (size_t j = 0; j < midline.size(); ++j) {
            const Point& midline_point = midline[j];
            float dx = point.x - midline_point.x;
            float dy = point.y - midline_point.y;
            float dist = sqrtf(dx * dx + dy * dy);

            if (dist < min_dist) {
                min_dist = dist;
                closest_midline_point = midline_point;
            }
        }

        if (min_dist <= TRACK_WIDTH / 2) {
            float dx = point.x - closest_midline_point.x;
            std::string label = (dx > 0) ? "blue" : "yellow";
            labeled_cones.emplace_back(point, label);
        }
    }

    return labeled_cones;
}