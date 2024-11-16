#ifndef DBSCAN_CUH
#define DBSCAN_CUH

#include <cuda_runtime.h>
#include "Point.hpp" // Assuming this defines the Point structure

// CUDA Kernels and Helper Declarations (Defined in DBscan.cu)
__global__ void initializeClusters(int* parent, int num_points);
__device__ int findRoot(int* parent, int idx);
__global__ void findAndUnionClusters(const Point* points, int* parent, int num_points, float eps);
__global__ void flattenClusters(int* parent, int num_points);
__global__ void computeCentroids(const Point* points, const int* parent, Point* centroids, int* cluster_sizes, int num_points);
__global__ void finalizeCentroids(Point* centroids, int* cluster_sizes, int num_points);

#endif // DBSCAN_CUH