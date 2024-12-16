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
const float TRACK_WIDTH = 3.0f; // Approximate track width for labeling

// Filter points with x >= 0
std::vector<Point> filterPointsByY(const std::vector<Point>& points) {
    std::vector<Point> filtered_points;
    for (const auto& point : points) {
        if (point.y >= 0) {
            filtered_points.push_back(point);
        }
    }
    return filtered_points;
}

// Sort points along the x-axis
std::vector<Point> sortPointsByX(const std::vector<Point>& points) {
    std::vector<Point> sorted_points = points;
    std::sort(sorted_points.begin(), sorted_points.end(), [](const Point& a, const Point& b) {
        return a.y < b.y;
    });
    return sorted_points;
}


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



// Helper Struct for Spline Coefficients
struct SplineSegment {
    float a, b, c, d; // Coefficients for the cubic polynomial: y = a + b*(x-xi) + c*(x-xi)^2 + d*(x-xi)^3
    float x;          // x-coordinate of the segment start
};

// Sort points along a chosen axis ('x' or 'y')
std::vector<Point> sortPointsByAxis(const std::vector<Point>& points, const std::vector<int>& cluster, char axis) {
    std::vector<Point> sorted_points;
    for (int idx : cluster) {
        sorted_points.push_back(points[idx]);
    }

    if (axis == 'x') {
        std::sort(sorted_points.begin(), sorted_points.end(), [](const Point& a, const Point& b) {
            return a.x < b.x;
        });
    } else if (axis == 'y') {
        std::sort(sorted_points.begin(), sorted_points.end(), [](const Point& a, const Point& b) {
            return a.y < b.y;
        });
    }
    return sorted_points;
}

// Function to fit a cubic spline
std::vector<SplineSegment> fitCubicSpline(const std::vector<float>& xs, const std::vector<float>& ys) {
    int n = xs.size();
    if (n < 2) {
        throw std::invalid_argument("Need at least two points for spline fitting.");
    }

    std::vector<float> h(n - 1), alpha(n - 1), l(n), mu(n), z(n);
    std::vector<float> b(n - 1), c(n), d(n - 1);
    std::vector<SplineSegment> spline_segments;

    // Step 1: Compute h and alpha
    for (int i = 0; i < n - 1; ++i) {
        h[i] = xs[i + 1] - xs[i];
        alpha[i] = (3.0f / h[i]) * (ys[i + 1] - ys[i]) - (3.0f / h[i - 1]) * (ys[i] - ys[i - 1]);
    }

    // Step 2: Solve tridiagonal system
    l[0] = 1.0f;
    mu[0] = 0.0f;
    z[0] = 0.0f;

    for (int i = 1; i < n - 1; ++i) {
        l[i] = 2.0f * (xs[i + 1] - xs[i - 1]) - h[i - 1] * mu[i - 1];
        mu[i] = h[i] / l[i];
        z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
    }

    l[n - 1] = 1.0f;
    z[n - 1] = 0.0f;
    c[n - 1] = 0.0f;

    // Step 3: Back substitution
    for (int i = n - 2; i >= 0; --i) {
        c[i] = z[i] - mu[i] * c[i + 1];
        b[i] = (ys[i + 1] - ys[i]) / h[i] - h[i] * (c[i + 1] + 2.0f * c[i]) / 3.0f;
        d[i] = (c[i + 1] - c[i]) / (3.0f * h[i]);
    }

    // Step 4: Store coefficients in spline segments
    for (int i = 0; i < n - 1; ++i) {
        SplineSegment segment = {ys[i], b[i], c[i], d[i], xs[i]};
        spline_segments.push_back(segment);
    }

    return spline_segments;
}

// Function to evaluate spline at a given x
float evaluateSpline(const std::vector<SplineSegment>& spline_segments, float x) {
    for (const auto& segment : spline_segments) {
        if (x >= segment.x && x <= segment.x + segment.a) {
            float dx = x - segment.x;
            return segment.a + segment.b * dx + segment.c * dx * dx + segment.d * dx * dx * dx;
        }
    }
    throw std::out_of_range("X value is outside the range of the spline.");
}

// Compute midline with constraints
std::vector<Point> computeMidlineWithOrigin(const std::vector<Point>& points, const std::vector<int>& cluster) {
    // Filter points with x >= 0 and y >= 0
    std::vector<Point> filtered_points;
    for (int idx : cluster) {
        if (points[idx].x >= 0 && points[idx].y >= 0) {
            filtered_points.push_back(points[idx]);
        }
    }

    // Sort filtered points by x
    std::vector<Point> sorted_points = sortPointsByX(filtered_points);

    // Add origin (0, 0, 0) as the first point in the midline
    std::vector<Point> midline = {Point(0.0f, 0.0f, 0.0f, 0.0f)};

    // Extract x and y values
    std::vector<float> xs, ys;
    for (const auto& point : sorted_points) {
        xs.push_back(point.x);
        ys.push_back(point.y);
    }

    // Fit cubic spline
    auto spline_segments = fitCubicSpline(xs, ys);

    // Sample the spline at regular intervals
    float step = 0.1f; // Interval for sampling
    for (float x = xs.front(); x <= xs.back(); x += step) {
        float y = evaluateSpline(spline_segments, x);
        midline.emplace_back(Point(x, y, 0.0f, 0.0f));
    }

    return midline;
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



std::vector<std::pair<Point, std::string>> labelCones(const std::vector<Point>& points, const std::vector<Point>& midline, const std::vector<int>& cluster) {
    std::vector<std::pair<Point, std::string>> labeled_cones;

    for (int idx : cluster) {
        const auto& point = points[idx];
        float min_dist = INFINITY;
        Point closest_midline_point;
        size_t closest_midline_idx = 0;

        // Find the closest midline point and its index
        for (size_t j = 0; j < midline.size(); ++j) {
            const auto& midline_point = midline[j];
            float dx = point.x - midline_point.x;
            float dy = point.y - midline_point.y;
            float dist = sqrtf(dx * dx + dy * dy);

            if (dist < min_dist) {
                min_dist = dist;
                closest_midline_point = midline_point;
                closest_midline_idx = j;
            }
        }

        // Compute the local normal to the midline
        float normal_x = 0, normal_y = 0;
        if (closest_midline_idx < midline.size() - 1) {
            // Use the next midline point for direction vector
            const auto& next_point = midline[closest_midline_idx + 1];
            float dir_x = next_point.x - closest_midline_point.x;
            float dir_y = next_point.y - closest_midline_point.y;

            // Normal vector (perpendicular to direction)
            normal_x = -dir_y;
            normal_y = dir_x;
        } else if (closest_midline_idx > 0) {
            // Use the previous midline point for direction vector
            const auto& prev_point = midline[closest_midline_idx - 1];
            float dir_x = closest_midline_point.x - prev_point.x;
            float dir_y = closest_midline_point.y - prev_point.y;

            // Normal vector (perpendicular to direction)
            normal_x = -dir_y;
            normal_y = dir_x;
        }

        // Normalize the normal vector
        float norm = sqrtf(normal_x * normal_x + normal_y * normal_y);
        if (norm > 1e-6) { // Avoid division by zero
            normal_x /= norm;
            normal_y /= norm;
        }

        // Project the cone onto the normal to determine its side
        float rel_x = point.x - closest_midline_point.x;
        float rel_y = point.y - closest_midline_point.y;
        float projection = rel_x * normal_x + rel_y * normal_y;

        // Label based on the projection's sign
        std::string label = (projection > 0) ? "blue" : "yellow";
        labeled_cones.emplace_back(point, label);
    }

    return labeled_cones;
}


// Main pipeline
// Main pipeline
// Main pipeline
std::vector<std::pair<Point, std::string>> ClusterCones(const std::vector<Point>& points) {
    // Step 1: Filter points with x < 0 or y < 0
    auto valid_points = filterPointsByY(points);

    // Step 2: Run DBSCAN with large eps using the new implementation
    auto clusters = NewRunDBSCAN(valid_points, EPS, MIN_SAMPLES);

    // Step 3: Find the cluster closest to the origin
    int best_cluster_idx = -1;
    float min_dist_to_origin = INFINITY;

    for (size_t i = 0; i < clusters.size(); ++i) {
        for (size_t j = 0; j < clusters[i].size(); ++j) {
            int idx = clusters[i][j];
            const Point& p = valid_points[idx];
            float dist_to_origin = sqrtf(p.x * p.x + p.y * p.y);
            if (dist_to_origin < min_dist_to_origin) {
                min_dist_to_origin = dist_to_origin;
                best_cluster_idx = i;
            }
        }
    }

    // If no valid cluster is found, return an empty result
    if (best_cluster_idx == -1) {
        return {};
    }

    // Step 4: Compute midline for the selected cluster
    const auto& best_cluster = clusters[best_cluster_idx];
    std::vector<Point> midline = computeMidlineWithOrigin(valid_points, best_cluster);

    // Step 5: Use the labelCones function to label the cones
    auto labeled_cones = labelCones(valid_points, midline, best_cluster);

       // DEBUG
    for (const auto& midline_point : midline) {
        labeled_cones.emplace_back(midline_point, "green");
    }

    return labeled_cones;
}