#include <cuda_runtime.h>
#include <vector>
#include <iostream>
#include <cmath>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include "Point.hpp"
#include "DBscan.cuh"

// Helper function to calculate the number of grid blocks
inline dim3 calculateGridSize(int num_points, int block_size) {
    return dim3((num_points + block_size - 1) / block_size);
}

// Parallelized DBSCAN implementation
std::vector<int> runParallelDBSCAN(const std::vector<Point>& points, float eps, int min_samples) {
    int num_points = points.size();

    // Allocate device memory
    thrust::device_vector<Point> d_points(points.begin(), points.end());
    thrust::device_vector<int> d_parent(num_points);
    thrust::device_vector<int> d_labels(num_points, -1); // -1 indicates noise
    thrust::device_vector<int> d_cluster_sizes(num_points, 0);
    thrust::device_vector<int> d_is_core(num_points, 0); // 1 if point is core

    // Step 1: Initialize clusters
    dim3 block(256);
    dim3 grid = calculateGridSize(num_points, block.x);
    initializeClusters<<<grid, block>>>(thrust::raw_pointer_cast(d_parent.data()), num_points);
    cudaDeviceSynchronize();

    // Step 2: Find and union clusters
    findAndUnionClusters<<<grid, block>>>(
        thrust::raw_pointer_cast(d_points.data()),
        thrust::raw_pointer_cast(d_parent.data()),
        num_points, eps);
    cudaDeviceSynchronize();

    // Step 3: Flatten clusters for path compression
    flattenClusters<<<grid, block>>>(thrust::raw_pointer_cast(d_parent.data()), num_points);
    cudaDeviceSynchronize();

    // Step 4: Assign cluster IDs
    thrust::host_vector<int> h_parent = d_parent; // Copy parents back to host
    std::unordered_map<int, int> cluster_map;
    int next_cluster_id = 0;

    for (int i = 0; i < num_points; ++i) {
        int root = h_parent[i];
        if (cluster_map.find(root) == cluster_map.end()) {
            cluster_map[root] = next_cluster_id++;
        }
        h_parent[i] = cluster_map[root];
    }

    // Copy back updated cluster IDs to the device
    thrust::device_vector<int> d_cluster_ids(h_parent.begin(), h_parent.end());

    // Copy cluster IDs to host for returning
    thrust::host_vector<int> h_cluster_ids = d_cluster_ids;

    return std::vector<int>(h_cluster_ids.begin(), h_cluster_ids.end());
}

// Parallelized main pipeline
std::vector<std::pair<Point, std::string>> ColorCones(const std::vector<Point>& points) {
    auto start = std::chrono::high_resolution_clock::now();
    
    int num_points = points.size();

    // Step 1: Run DBSCAN to identify clusters
    auto cluster_ids = runParallelDBSCAN(points, 5.5f, 3);

    // Step 2: Find the cluster closest to the origin
    std::unordered_map<int, Point> cluster_centroids;
    std::unordered_map<int, int> cluster_sizes;
    int best_cluster_id = -1;
    float min_dist_to_origin = INFINITY;

    for (int i = 0; i < num_points; ++i) {
        int cluster_id = cluster_ids[i];
        if (cluster_id >= 0) {
            const Point& p = points[i];
            float dist_to_origin = sqrtf(p.x * p.x + p.y * p.y);

            // Update cluster centroid calculations
            cluster_centroids[cluster_id].x += p.x;
            cluster_centroids[cluster_id].y += p.y;
            cluster_sizes[cluster_id]++;

            // Track the closest cluster to the origin
            if (dist_to_origin < min_dist_to_origin) {
                min_dist_to_origin = dist_to_origin;
                best_cluster_id = cluster_id;
            }
        }
    }

    if (best_cluster_id == -1) {
        throw std::runtime_error("No valid cluster found closest to the origin.");
    }

    // Compute final centroids
    for (auto& [id, centroid] : cluster_centroids) {
        int size = cluster_sizes[id];
        centroid.x /= size;
        centroid.y /= size;
    }

    // Extract points belonging to the best cluster
    std::vector<Point> track_points;
    for (int i = 0; i < num_points; ++i) {
        if (cluster_ids[i] == best_cluster_id) {
            track_points.push_back(points[i]);
        }
    }

    // Step 3: Sub-cluster the track points with smaller EPS
    float small_eps = 2.8f;
    int min_samples_for_coloring = 2;
    auto sub_cluster_ids = runParallelDBSCAN(track_points, small_eps, min_samples_for_coloring);

    // Step 4: Assign labels based on sub-cluster membership
    std::vector<std::pair<Point, std::string>> labeled_cones;
    std::unordered_map<int, int> sub_cluster_sizes;

    for (int id : sub_cluster_ids) {
        if (id >= 0) {
            sub_cluster_sizes[id]++;
        }
    }

    if (sub_cluster_sizes.size() < 2) {
        throw std::runtime_error("Not enough sub-clusters found for left/right labeling.");
    }

    // Find two largest sub-clusters
    std::vector<std::pair<int, int>> sorted_clusters(sub_cluster_sizes.begin(), sub_cluster_sizes.end());
    std::sort(sorted_clusters.begin(), sorted_clusters.end(), [](const auto& a, const auto& b) {
        return b.second < a.second; // Sort descending by size
    });

    int left_cluster_id = sorted_clusters[0].first;
    int right_cluster_id = sorted_clusters[1].first;

    for (int i = 0; i < track_points.size(); ++i) {
        int sub_cluster_id = sub_cluster_ids[i];
        if (sub_cluster_id == left_cluster_id) {
            labeled_cones.emplace_back(track_points[i], "blue");
        } else if (sub_cluster_id == right_cluster_id) {
            labeled_cones.emplace_back(track_points[i], "yellow");
        }
    }


    // Stop the timer
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed_time = end - start;

    // Print the timing results
    std::cout << "Clustered coloring execution time: " << elapsed_time.count() << " ms" << std::endl;

    return labeled_cones;
}