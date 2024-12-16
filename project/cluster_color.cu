#include <vector>
#include <cmath>
#include <string>
#include <algorithm>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include "Point.hpp"
#include <algorithm>
#include <stdexcept>

// Global constants
const float EPS = 5.5f;       // Large `eps` for clustering
const int MIN_SAMPLES = 3;    // Minimum points for a valid cluster

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


std::vector<std::pair<Point, std::string>> labelConesWithDBSCAN(
    const std::vector<Point>& points,
    const std::vector<int>& cluster,
    float eps_for_coloring,
    int min_samples_for_coloring) {

    // Extract the points in the "track" cluster
    std::vector<Point> track_points;
    for (int idx : cluster) {
        track_points.push_back(points[idx]);
    }

    // Run DBSCAN again with smaller EPS
    auto sub_clusters = NewRunDBSCAN(track_points, eps_for_coloring, min_samples_for_coloring);

    // Debug: Print number of sub-clusters found
    std::cout << "Sub-clusters found in track cluster: " << sub_clusters.size() << std::endl;

    if (sub_clusters.size() < 2) {
        throw std::runtime_error("Not enough sub-clusters found for labeling as left/right bounds.");
    }

    // If more than two clusters, find the two closest to the origin
    if (sub_clusters.size() > 2) {
        std::vector<std::pair<float, int>> cluster_distances; // (distance_to_origin, cluster_index)

        for (size_t i = 0; i < sub_clusters.size(); ++i) {
            float min_dist_to_origin = INFINITY;
            for (int idx : sub_clusters[i]) {
                const Point& p = track_points[idx];
                float dist_to_origin = sqrtf(p.x * p.x + p.y * p.y);
                min_dist_to_origin = std::min(min_dist_to_origin, dist_to_origin);
            }
            cluster_distances.emplace_back(min_dist_to_origin, i);
        }

        // Sort clusters by distance to origin
        std::sort(cluster_distances.begin(), cluster_distances.end());

        // Select the two closest clusters
        sub_clusters = {sub_clusters[cluster_distances[0].second], sub_clusters[cluster_distances[1].second]};
    }

    // Assign labels to the two clusters
    std::vector<std::pair<Point, std::string>> labeled_cones;

    // Ensure consistent ordering: label the cluster with smaller mean x-coordinate as "yellow"
    float mean_x_cluster_0 = 0, mean_x_cluster_1 = 0;
    for (int idx : sub_clusters[0]) {
        mean_x_cluster_0 += track_points[idx].x;
    }
    for (int idx : sub_clusters[1]) {
        mean_x_cluster_1 += track_points[idx].x;
    }
    mean_x_cluster_0 /= sub_clusters[0].size();
    mean_x_cluster_1 /= sub_clusters[1].size();

    bool cluster_0_is_yellow = mean_x_cluster_0 < mean_x_cluster_1;

    for (size_t i = 0; i < sub_clusters.size(); ++i) {
        std::string label = (cluster_0_is_yellow == (i == 0)) ? "yellow" : "blue";
        for (int idx : sub_clusters[i]) {
            labeled_cones.emplace_back(track_points[idx], label);
        }
    }

    return labeled_cones;
}


// Main pipeline
std::vector<std::pair<Point, std::string>> ColorCones(const std::vector<Point>& points) {
    // Step 1: Run DBSCAN with large EPS to identify the track cluster
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
        throw std::runtime_error("No valid cluster found closest to the origin.");
    }

    // Step 3: Re-run DBSCAN on the identified "track" cluster with smaller EPS
    const auto& track_cluster = clusters[best_cluster_idx];
    float small_eps = 2.8f; // Smaller EPS for sub-clustering
    int min_samples_for_coloring = 2; // Minimum samples for sub-clustering

    auto labeled_cones = labelConesWithDBSCAN(points, track_cluster, small_eps, min_samples_for_coloring);

    return labeled_cones;
}