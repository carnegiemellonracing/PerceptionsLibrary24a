// #include <cuda_runtime.h>
// #include <vector>
// #include <cmath>
// #include <algorithm>
// #include <thrust/device_vector.h>
// #include <thrust/host_vector.h>
// #include <iostream>
// #include <unordered_map>
// #include "Point.hpp"


// // Was 10
// #define SECTION_RADIUS 10.0f
// #define SPLINE_SAMPLES 1000
// #define TRACK_WIDTH 3.0f



// struct SplineSegment {
//     float a, b, c, d; // Coefficients for the cubic spline
//     float x_start;    // Start of the segment
// };



// std::vector<std::vector<int>> NewRunDBSCAN(const std::vector<Point>& points, float eps, int min_samples) {
//     int num_points = points.size();
//     std::vector<std::vector<int>> clusters;

//     // Create visited and labels arrays
//     std::vector<bool> visited(num_points, false);
//     std::vector<int> labels(num_points, -1);  // -1 means noise
//     int cluster_id = 0;

//     // Helper lambda to calculate distance
//     auto distance = [](const Point& a, const Point& b) {
//         float dx = a.x - b.x;
//         float dy = a.y - b.y;
//         float dz = a.z - b.z;
//         return sqrtf(dx * dx + dy * dy + dz * dz);
//     };

//     // Helper function to find neighbors
//     auto get_neighbors = [&](int idx) {
//         std::vector<int> neighbors;
//         for (int i = 0; i < num_points; ++i) {
//             if (i != idx && distance(points[idx], points[i]) <= eps) {
//                 neighbors.push_back(i);
//             }
//         }
//         return neighbors;
//     };

//     // Main DBSCAN logic
//     for (int i = 0; i < num_points; ++i) {
//         if (visited[i]) continue;
//         visited[i] = true;

//         // Find neighbors
//         auto neighbors = get_neighbors(i);
//         if (neighbors.size() < min_samples) {
//             labels[i] = -1;  // Mark as noise
//             continue;
//         }

//         // Create a new cluster
//         clusters.emplace_back();
//         labels[i] = cluster_id;
//         clusters[cluster_id].push_back(i);

//         // Expand the cluster
//         std::vector<int> to_process = neighbors;
//         while (!to_process.empty()) {
//             int current = to_process.back();
//             to_process.pop_back();

//             if (!visited[current]) {
//                 visited[current] = true;

//                 // Get neighbors of the current point
//                 auto current_neighbors = get_neighbors(current);
//                 if (current_neighbors.size() >= min_samples) {
//                     to_process.insert(to_process.end(), current_neighbors.begin(), current_neighbors.end());
//                 }
//             }

//             if (labels[current] == -1 || labels[current] == -2) {
//                 labels[current] = cluster_id;
//                 clusters[cluster_id].push_back(current);
//             }
//         }

//         cluster_id++;
//     }

//     return clusters;
// }


// __global__ void computeMidlinePoints(const Point* sections, Point* midline, const int* section_sizes, int num_sections) {
//     int idx = blockIdx.x * blockDim.x + threadIdx.x;
//     if (idx >= num_sections) return;

//     const Point* section_start = sections + idx * section_sizes[idx];
//     int section_size = section_sizes[idx];

//     float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
//     for (int i = 0; i < section_size; ++i) {
//         sum_x += section_start[i].x;
//         sum_y += section_start[i].y;
//         sum_z += section_start[i].z;
//     }

//     midline[idx] = {sum_x / section_size, sum_y / section_size, sum_z / section_size, 0.0f};
// }




// // CUDA kernel to classify cones based on midline
// __global__ void classifyCones(const Point* cones, const Point* midline, int* labels, int num_cones, int num_midline_points) {
//     int idx = blockIdx.x * blockDim.x + threadIdx.x;
//     if (idx >= num_cones) return;

//     const Point& cone = cones[idx];
//     float min_dist = INFINITY;
//     int closest_segment = -1;

//     // Find the closest segment of the midline
//     for (int i = 0; i < num_midline_points - 1; ++i) {
//         float x1 = midline[i].x, y1 = midline[i].y;
//         float x2 = midline[i + 1].x, y2 = midline[i + 1].y;

//         // Calculate distance from the cone to the line segment
//         float dx = x2 - x1, dy = y2 - y1;
//         float t = ((cone.x - x1) * dx + (cone.y - y1) * dy) / (dx * dx + dy * dy);
//         t = fmaxf(0.0f, fminf(1.0f, t)); // Clamp t to [0, 1]
//         float proj_x = x1 + t * dx, proj_y = y1 + t * dy;
//         float dist = sqrtf((cone.x - proj_x) * (cone.x - proj_x) + (cone.y - proj_y) * (cone.y - proj_y));

//         if (dist < min_dist) {
//             min_dist = dist;
//             closest_segment = i;
//         }
//     }

//     // Classify based on relative position to the closest segment
//     const Point& p1 = midline[closest_segment];
//     const Point& p2 = midline[closest_segment + 1];
//     float normal_x = -(p2.y - p1.y), normal_y = p2.x - p1.x; // Perpendicular vector
//     float dx = cone.x - p1.x, dy = cone.y - p1.y;

//     labels[idx] = (dx * normal_x + dy * normal_y > 0) ? 1 : 0; // 1 for blue, 0 for yellow
// }





// std::vector<std::pair<Point, std::string>> ColorCones(const std::vector<Point>& points) {


//     // Start the timer
//     auto start_time = std::chrono::high_resolution_clock::now();


//     // Step 1: Run DBSCAN to find clusters
//     auto clusters = NewRunDBSCAN(points, 7, 3);

//     // Step 2: Find the cluster closest to the origin
//     int best_cluster_idx = -1;

//     // Compute closest distance for each cluster in parallel
//     thrust::host_vector<float> min_distances(clusters.size());
//     thrust::host_vector<int> cluster_indices(clusters.size());

//     thrust::transform(
//         thrust::counting_iterator<int>(0),
//         thrust::counting_iterator<int>(clusters.size()),
//         min_distances.begin(),
//         [&](int cluster_idx) {
//             float min_dist = INFINITY;
//             for (int idx : clusters[cluster_idx]) {
//                 const Point& p = points[idx];
//                 float dist_to_origin = sqrtf(p.x * p.x + p.y * p.y);
//                 if (dist_to_origin < min_dist) {
//                     min_dist = dist_to_origin;
//                     cluster_indices[cluster_idx] = cluster_idx;
//                 }
//             }
//             return min_dist;
//         }
//     );

//     // Find the cluster closest to the origin
//     best_cluster_idx = thrust::min_element(min_distances.begin(), min_distances.end()) - min_distances.begin();

//     if (best_cluster_idx == -1) {
//         throw std::runtime_error("No valid cluster found closest to the origin.");
//     }

//     // Step 3: make preprocessed points
//     std::vector<Point> preprocessed_points;
//     for (int idx : clusters[best_cluster_idx]) {
//         preprocessed_points.push_back(points[idx]);
//     }

//     int num_points = preprocessed_points.size();

//     Point closest_positive_x, closest_negative_x;
//     float min_positive_x_dist = INFINITY, min_negative_x_dist = INFINITY;

//     // Step 4: Find the starting reference point
//     for (const auto& p : preprocessed_points) {
//         float dist = sqrtf(p.x * p.x + p.y * p.y);
//         if (p.x > 0 && dist < min_positive_x_dist) {
//             min_positive_x_dist = dist;
//             closest_positive_x = p;
//         } else if (p.x < 0 && dist < min_negative_x_dist) {
//             min_negative_x_dist = dist;
//             closest_negative_x = p;
//         }
//     }

//     if (min_positive_x_dist == INFINITY || min_negative_x_dist == INFINITY) {
//         std::cerr << "Fallback: Unable to find points in positive or negative x. Searching for fallback points.\n";

//         // Find the absolute closest point to the origin
//         Point closest1;
//         float min_dist1 = INFINITY;

//         for (const auto& p : preprocessed_points) {
//             float dist = sqrtf(p.x * p.x + p.y * p.y);
//             if (dist < min_dist1) {
//                 min_dist1 = dist;
//                 closest1 = p;
//             }
//         }

//         // Find the next closest point that is at least 1.5m away from closest1
//         Point closest2;
//         float min_dist2 = INFINITY;

//         for (const auto& p : preprocessed_points) {
//             float dist_to_origin = sqrtf(p.x * p.x + p.y * p.y);
//             float dist_to_closest1 = sqrtf(
//                 (p.x - closest1.x) * (p.x - closest1.x) +
//                 (p.y - closest1.y) * (p.y - closest1.y)
//             );

//             if (dist_to_origin < min_dist2 && dist_to_closest1 >= 1.5f) {
//                 min_dist2 = dist_to_origin;
//                 closest2 = p;
//             }
//         }

//         if (min_dist2 == INFINITY) {
//             throw std::runtime_error("Fallback method failed: No second point found that is at least 1.5m away from the closest point.");
//         }

//         // Set fallback points as closest_positive_x and closest_negative_x
//         closest_positive_x = closest1;
//         closest_negative_x = closest2;

//         std::cerr << "Fallback successful: Closest points found are spaced more than 1.5m apart.\n";
//     }

//     Point current_ref = {
//         (closest_positive_x.x + closest_negative_x.x) / 2.0f,
//         (closest_positive_x.y + closest_negative_x.y) / 2.0f,
//         (closest_positive_x.z + closest_negative_x.z) / 2.0f,
//         0.0f
//     };



//     // Step 5: Section points and record the start and end points
//     std::vector<std::vector<Point>> sections;
//     std::vector<Point> section_start_points;
//     std::vector<Point> remaining_points = preprocessed_points;

//     while (!remaining_points.empty()) {
//         std::vector<Point> current_section;
//         auto it = remaining_points.begin();
//         while (it != remaining_points.end()) {
//             float dx = it->x - current_ref.x, dy = it->y - current_ref.y;
//             float dist_to_current_section = sqrtf(dx * dx + dy * dy);

//             // Check overlap with previous section centers
//             float dist_to_previous_section = INFINITY;
//             if (!sections.empty()) {
//                 const Point& previous_center = section_start_points.back();
//                 float dx_prev = it->x - previous_center.x, dy_prev = it->y - previous_center.y;
//                 dist_to_previous_section = sqrtf(dx_prev * dx_prev + dy_prev * dy_prev);
//             }

//             // Assign point only if it's closer to the current section center
//             if (dist_to_current_section <= SECTION_RADIUS &&
//                 dist_to_current_section < dist_to_previous_section) {
//                 current_section.push_back(*it);
//                 it = remaining_points.erase(it);
//             } else {
//                 ++it;
//             }
//         }

//         if (current_section.empty()) break;

//         sections.push_back(current_section);
//         section_start_points.push_back(current_ref);

//         // Update the reference point
//         if (!current_section.empty()) {
//             float max_dist1 = 0.0f, max_dist2 = 0.0f;
//             Point furthest1 = current_section[0], furthest2 = current_section[0];
//             for (const auto& point : current_section) {
//                 float dist = sqrtf((point.x - current_ref.x) * (point.x - current_ref.x) +
//                                 (point.y - current_ref.y) * (point.y - current_ref.y));
//                 if (dist > max_dist1) {
//                     max_dist2 = max_dist1;
//                     furthest2 = furthest1;
//                     max_dist1 = dist;
//                     furthest1 = point;
//                 } else if (dist > max_dist2) {
//                     max_dist2 = dist;
//                     furthest2 = point;
//                 }
//             }
//             current_ref = {
//                 (furthest1.x + furthest2.x) / 2.0f,
//                 (furthest1.y + furthest2.y) / 2.0f,
//                 (furthest1.z + furthest2.z) / 2.0f,
//                 0.0f
//             };
//         }
//     }

//     // Flatten sections into a single array for midline computation
//     std::vector<Point> flattened_sections;
//     std::vector<int> section_sizes;
//     for (const auto& section : sections) {
//         flattened_sections.insert(flattened_sections.end(), section.begin(), section.end());
//         section_sizes.push_back(section.size());
//     }

//     // Compute midline points
//     thrust::device_vector<Point> d_sections(flattened_sections);
//     thrust::device_vector<int> d_section_sizes(section_sizes);
//     thrust::device_vector<Point> d_midline(sections.size());

//     computeMidlinePoints<<<(sections.size() + 255) / 256, 256>>>(
//         thrust::raw_pointer_cast(d_sections.data()),
//         thrust::raw_pointer_cast(d_midline.data()),
//         thrust::raw_pointer_cast(d_section_sizes.data()),
//         sections.size());
//     cudaDeviceSynchronize();

//     thrust::host_vector<Point> midline = d_midline;

//     // Add the first and last points to the midline
//     midline.insert(midline.begin(), section_start_points.front()); // Add start point
//     midline.push_back(current_ref); // Add end point

//     // Classify cones based on the updated midline
//     thrust::device_vector<int> d_labels(num_points);
//     thrust::device_vector<Point> d_points(preprocessed_points);
//     thrust::device_vector<Point> d_updated_midline(midline);
//     classifyCones<<<(num_points + 255) / 256, 256>>>(
//         thrust::raw_pointer_cast(d_points.data()),
//         thrust::raw_pointer_cast(d_updated_midline.data()),
//         thrust::raw_pointer_cast(d_labels.data()),
//         num_points, midline.size());
//     cudaDeviceSynchronize();

//     thrust::host_vector<int> labels = d_labels;

//     // Stop the timer
//     auto end_time = std::chrono::high_resolution_clock::now();
//     auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
//     std::cout << "ColorCones execution time: " << duration << " µs" << std::endl;

//     // Create output with midline points labeled as "green" and section start points labeled as "section"
//     std::vector<std::pair<Point, std::string>> classified_cones;
//     for (int i = 0; i < num_points; ++i) {
//         classified_cones.emplace_back(preprocessed_points[i], (labels[i] == 1) ? "blue" : "yellow");
//     }
//     for (const auto& p : midline) {
//         classified_cones.emplace_back(p, "green");
//     }
//     for (const auto& p : section_start_points) {
//         classified_cones.emplace_back(p, "section");
//     }

//     return classified_cones;
// }






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