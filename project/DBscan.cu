// Implementation History:
// Was working on Pi-Lisco, now switched ot DBSCAN, but Pi-Lisco could be good
// Started with DBscan
// Went forward with a modified dbscan with a tree based clustering method, works quite well



#include <cuda_runtime.h>
#include <vector>
#include <iostream>
#include <cmath>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/fill.h>
#include <unordered_map>
#include "Point.hpp"
#include "DBscan.cuh"


// CUDA kernel for union-find initialization
__global__ void initializeClusters(int* parent, int num_points) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_points) return;
    parent[idx] = idx; // Each point starts as its own cluster
}

// Device function to find the root of a cluster with path compression
__device__ int findRoot(int* parent, int idx) {
    while (idx != parent[idx]) {
        parent[idx] = parent[parent[idx]]; // Path compression
        idx = parent[idx];
    }
    return idx;
}

// CUDA kernel to find neighbors and union clusters
__global__ void findAndUnionClusters(
    const Point* points, int* parent, int num_points, float eps) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_points) return;

    for (int j = 0; j < num_points; ++j) {
        if (idx == j) continue;

        float dx = points[idx].x - points[j].x;
        float dy = points[idx].y - points[j].y;
        float dz = points[idx].z - points[j].z;
        float dist = sqrtf(dx * dx + dy * dy + dz * dz);

        if (dist <= eps) {
            // Union clusters
            int root1 = findRoot(parent, idx);
            int root2 = findRoot(parent, j);
            if (root1 != root2) {
                atomicMin(&parent[root1], root2); // Union by assigning the smaller root
                atomicMin(&parent[root2], root1);
            }
        }
    }
}

// CUDA kernel to compute centroids for clusters
__global__ void computeCentroids(
    const Point* points, const int* parent, Point* centroids, int* cluster_sizes, int num_points) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_points) return;

    int root = findRoot(const_cast<int*>(parent), idx); // Find cluster root
    atomicAdd(&centroids[root].x, points[idx].x);
    atomicAdd(&centroids[root].y, points[idx].y);
    atomicAdd(&centroids[root].z, points[idx].z);
    atomicAdd(&centroids[root].intensity, points[idx].intensity);
    atomicAdd(&cluster_sizes[root], 1);
}

// CUDA kernel to finalize centroids
__global__ void finalizeCentroids(Point* centroids, int* cluster_sizes, int num_points) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_points) return;

    int size = cluster_sizes[idx];
    if (size > 0) {
        centroids[idx].x /= size;
        centroids[idx].y /= size;
        centroids[idx].z /= size;
        centroids[idx].intensity /= size;
    }
}

__global__ void flattenClusters(int* parent, int num_points) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= num_points) return;

    parent[idx] = findRoot(parent, idx); // Compress paths fully
}



std::vector<Point> runDBSCAN(const std::vector<Point>& h_points, float eps, int min_samples) {
    int num_points = h_points.size();

    // Create CUDA events for timing
    cudaEvent_t total_start, total_stop, start, stop;
    cudaEventCreate(&total_start);
    cudaEventCreate(&total_stop);
    cudaEventCreate(&start);
    cudaEventCreate(&stop);

    float time_initialize = 0.0f, time_union = 0.0f, time_flatten = 0.0f, time_compute_centroids = 0.0f, time_finalize_centroids = 0.0f, total_time = 0.0f;

    // Record total start
    cudaEventRecord(total_start);

    // Allocate device memory
    thrust::device_vector<Point> d_points(h_points.begin(), h_points.end());
    thrust::device_vector<int> d_parent(num_points);
    thrust::device_vector<Point> d_centroids(num_points, Point());
    thrust::device_vector<int> d_cluster_sizes(num_points, 0);

    // Step 1: Initialize clusters
    cudaEventRecord(start);
    dim3 block(256);
    dim3 grid((num_points + block.x - 1) / block.x);
    initializeClusters<<<grid, block>>>(thrust::raw_pointer_cast(d_parent.data()), num_points);
    cudaDeviceSynchronize();
    cudaEventRecord(stop);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&time_initialize, start, stop);

    // Step 2: Find and union clusters
    cudaEventRecord(start);
    findAndUnionClusters<<<grid, block>>>(
        thrust::raw_pointer_cast(d_points.data()),
        thrust::raw_pointer_cast(d_parent.data()),
        num_points, eps);
    cudaDeviceSynchronize();
    cudaEventRecord(stop);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&time_union, start, stop);

    // Step 3: Flatten clusters
    cudaEventRecord(start);
    flattenClusters<<<grid, block>>>(
        thrust::raw_pointer_cast(d_parent.data()),
        num_points);
    cudaDeviceSynchronize();
    cudaEventRecord(stop);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&time_flatten, start, stop);

    // Copy flattened parent array back to host
    thrust::host_vector<int> h_parent = d_parent;

    // Identify unique roots
    std::unordered_map<int, int> root_to_index;
    std::vector<int> unique_roots;
    for (int i = 0; i < num_points; ++i) {
        int root = h_parent[i];
        if (root_to_index.find(root) == root_to_index.end()) {
            root_to_index[root] = unique_roots.size();
            unique_roots.push_back(root);
        }
    }

    // Step 4: Compute centroids
    cudaEventRecord(start);
    computeCentroids<<<grid, block>>>(
        thrust::raw_pointer_cast(d_points.data()),
        thrust::raw_pointer_cast(d_parent.data()),
        thrust::raw_pointer_cast(d_centroids.data()),
        thrust::raw_pointer_cast(d_cluster_sizes.data()),
        num_points);
    cudaDeviceSynchronize();
    cudaEventRecord(stop);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&time_compute_centroids, start, stop);

    // Step 5: Finalize centroids
    cudaEventRecord(start);
    finalizeCentroids<<<grid, block>>>(
        thrust::raw_pointer_cast(d_centroids.data()),
        thrust::raw_pointer_cast(d_cluster_sizes.data()),
        num_points);
    cudaDeviceSynchronize();
    cudaEventRecord(stop);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&time_finalize_centroids, start, stop);

    // Record total stop
    cudaEventRecord(total_stop);
    cudaEventSynchronize(total_stop);
    cudaEventElapsedTime(&total_time, total_start, total_stop);

    // Copy centroids back to host
    thrust::host_vector<Point> h_centroids = d_centroids;
    thrust::host_vector<int> h_cluster_sizes = d_cluster_sizes;

    // Filter out valid clusters using unique roots
    std::vector<Point> centroids;
    for (int root : unique_roots) {
        if (h_cluster_sizes[root] > 0) {
            centroids.push_back(h_centroids[root]);
        }
    }

    // Print timing for each step
    std::cout << "Timing Results (ms):" << std::endl;
    std::cout << " - Initialize Clusters: " << time_initialize << std::endl;
    std::cout << " - Find and Union Clusters: " << time_union << std::endl;
    std::cout << " - Flatten Clusters: " << time_flatten << std::endl;
    std::cout << " - Compute Centroids: " << time_compute_centroids << std::endl;
    std::cout << " - Finalize Centroids: " << time_finalize_centroids << std::endl;
    std::cout << " - Total Time: " << total_time << std::endl;

    // Clean up CUDA events
    cudaEventDestroy(start);
    cudaEventDestroy(stop);
    cudaEventDestroy(total_start);
    cudaEventDestroy(total_stop);

    return centroids;
}

