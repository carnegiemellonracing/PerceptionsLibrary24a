#include <iostream>
#include <vector>
#include <cmath>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "math.h"
#include "filter.h"
<<<<<<< HEAD
=======


>>>>>>> dc9189feef3b252f4d36b2efa99014106a5b5019

pcl::PointCloud<pcl::PointXYZ> filter::trim_cloud(pcl::PointCloud<pcl::PointXYZ> cloud) {
  double max_height = 5;
  double height = -1;
  double r_min = 1.5;
  double r_max = 10;
  double ang_cut = M_PI / 2;
  double scaling = 0.015;
}

pcl::PointCloud<pcl::PointXYZ> filter::remove_ground(pcl::PointCloud<pcl::PointXYZ> cloud) {
}

// Function implementing the GraceAndConrad algorithm
pcl::PointCloud<pcl::PointXYZ> GraceAndConrad(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
<<<<<<< HEAD
                                              double alpha, int num_bins, double height_threshold) {
                                                
    pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
    // Calculate angles and ranges
    std::vector<double> angles = {};
    std::vector<double> ranges = {};
    
    for (const auto& point : cloud->points) {
        double angle = std::atan2(point.y, point.x);
        if (angle < 0) 
            angle += 2 * M_PI;
        angles.push_back(angle);
=======
                                              pcl::PointCloud<pcl::PointXYZ>::Ptr points_ground,
                                              double alpha, int num_bins, double height_threshold) {
    pcl::PointCloud<pcl::PointXYZ> filtered_cloud;

    // Calculate angles and ranges
    std::vector<double> angles;
    std::vector<double> ranges;
    for (const auto& point : cloud->points) {
        double angle = std::atan2(point.y, point.x);
        if (angle < 0)
            angle += 2 * M_PI;
        angles.push_back(angle);

>>>>>>> dc9189feef3b252f4d36b2efa99014106a5b5019
        double range = std::sqrt(point.x * point.x + point.y * point.y);
        ranges.push_back(range);
    }

<<<<<<< HEAD
    // // Create angle bins
    double angle_min = *std::min_element(angles.begin(), angles.end());
    double angle_max = *std::max_element(angles.begin(), angles.end());
    int M = static_cast<int>((angle_max - angle_min) / alpha);
    // std::vector<double> gangles(M);
    // for (int i = 0; i < M; ++i) {
    //     gangles[i] = angle_min + i * alpha;
    // }
    // Map angles to segments
    std::vector<int> segments(angles.size());

    for (size_t i = 0; i < angles.size(); ++i) {
        segments[i] = static_cast<int>((angles[i] - angle_min) / alpha);
    }
=======
    // Create angle bins
    double angle_min = *std::min_element(angles.begin(), angles.end());
    double angle_max = *std::max_element(angles.begin(), angles.end());
    int M = static_cast<int>((angle_max - angle_min) / alpha);
    std::vector<double> gangles(M);
    for (int i = 0; i < M; i++) {
        gangles[i] = angle_min + i * alpha;
    }

    // Map angles to segments
    std::vector<int> segments(angles.size());
    for (size_t i = 0; i < angles.size(); i++) {
        segments[i] = static_cast<int>((angles[i] - angle_min) / alpha);
    }

>>>>>>> dc9189feef3b252f4d36b2efa99014106a5b5019
    // Create range bins
    double rmin = *std::min_element(ranges.begin(), ranges.end());
    double rmax = *std::max_element(ranges.begin(), ranges.end());
    double bin_size = (rmax - rmin) / num_bins;
<<<<<<< HEAD
    int num_bins = num_bins;
    std::vector<double> rbins(num_bins);
    for (int i = 0; i < num_bins; ++i) {
        rbins[i] = rmin + i * bin_size;
    }
    // Map ranges to regments
    std::vector<int> regments(ranges.size());
    for (size_t i = 0; i < ranges.size(); ++i) {
        regments[i] = static_cast<int>((ranges[i] - rmin) / bin_size);
    }
    // Calculate grid cell indices
    std::vector<int> grid_cell_indices(segments.size());
    for (size_t i = 0; i < segments.size(); ++i) {
        grid_cell_indices[i] = segments[i] * num_bins + regments[i];
    }
    // Process each segment
    for (int seg_idx = 0; seg_idx < M; ++seg_idx) {
        std::vector<std::pair<double, double>> Bines;
        std::vector<double> min_zs;
        for (int range_idx = 0; range_idx < num_bins; ++range_idx) {
            int bin_idx = seg_idx * num_bins + range_idx;
            // Get points in the current bin
            std::vector<int> idxs;
            for (size_t i = 0; i < grid_cell_indices.size(); ++i) {
=======
    int N = num_bins;
    std::vector<double> rbins(N);
    for (int i = 0; i < N; i++) {
        rbins[i] = rmin + i * bin_size;
    }

    // Map ranges to regments
    std::vector<int> regments(ranges.size());
    for (size_t i = 0; i < ranges.size(); i++) {
        regments[i] = static_cast<int>((ranges[i] - rmin) / bin_size);
    }

    // Calculate grid cell indices
    std::vector<int> grid_cell_indices(segments.size());
    for (size_t i = 0; i < segments.size(); i++) {
        grid_cell_indices[i] = segments[i] * N + regments[i];
    }

    // Process each segment
    for (int seg_idx = 0; seg_idx < M; seg_idx++) {
        std::vector<std::pair<double, double>> Bines;
        std::vector<double> min_zs;

        for (int range_idx = 0; range_idx < N; ++range_idx) {
            int bin_idx = seg_idx * N + range_idx;

            // Get points in the current bin
            std::vector<int> idxs;
            for (size_t i = 0; i < grid_cell_indices.size(); i++) {
>>>>>>> dc9189feef3b252f4d36b2efa99014106a5b5019
                if (grid_cell_indices[i] == bin_idx) {
                    idxs.push_back(i);
                }
            }
<<<<<<< HEAD
=======

>>>>>>> dc9189feef3b252f4d36b2efa99014106a5b5019
            if (!idxs.empty()) {
                // Find the minimum Z in the bin
                double min_z = cloud->points[idxs[0]].z;
                int min_idx = idxs[0];
                for (int idx : idxs) {
                    if (cloud->points[idx].z < min_z) {
                        min_z = cloud->points[idx].z;
                        min_idx = idx;
                    }
                }
<<<<<<< HEAD
=======

>>>>>>> dc9189feef3b252f4d36b2efa99014106a5b5019
                double range = ranges[min_idx];
                Bines.emplace_back(range, min_z);
                min_zs.push_back(min_z);
            }
        }
<<<<<<< HEAD
=======

>>>>>>> dc9189feef3b252f4d36b2efa99014106a5b5019
        // Linear regression on Bines
        if (Bines.size() >= 2) {
            size_t n = Bines.size();
            double sum_x = 0, sum_y = 0, sum_xx = 0, sum_xy = 0;
            for (const auto& p : Bines) {
                sum_x += p.first;
                sum_y += p.second;
                sum_xx += p.first * p.first;
                sum_xy += p.first * p.second;
            }
            double slope = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
            double intercept = (sum_y - slope * sum_x) / n;
<<<<<<< HEAD
            // Filter points in the segment
            for (size_t i = 0; i < cloud->points.size(); ++i) {
=======

            // Filter points in the segment
            for (size_t i = 0; i < cloud->points.size(); i++) {
>>>>>>> dc9189feef3b252f4d36b2efa99014106a5b5019
                if (segments[i] == seg_idx) {
                    double expected_z = slope * ranges[i] + intercept;
                    if (cloud->points[i].z > expected_z + height_threshold) {
                        filtered_cloud.points.push_back(cloud->points[i]);
                    }
                }
            }
        }
    }
<<<<<<< HEAD
    filtered_cloud.width = filtered_cloud.points.size();
    filtered_cloud.height = 1;
    filtered_cloud.is_dense = true;
    return filtered_cloud;
}
=======

    filtered_cloud.width = filtered_cloud.points.size();
    filtered_cloud.height = 1;
    filtered_cloud.is_dense = true;

    return filtered_cloud;
>>>>>>> dc9189feef3b252f4d36b2efa99014106a5b5019
}
pcl::PointCloud<pcl::PointXYZ> filter::section_pointcloud(pcl::PointCloud<pcl::PointXYZ> cloud,
                  int boxdim_x, int boxdim_y) {
}
pcl::PointCloud<pcl::PointXYZ> filter::fit_sections(pcl::PointCloud<pcl::PointXYZ> cloud) {
}
<<<<<<< HEAD
=======

>>>>>>> dc9189feef3b252f4d36b2efa99014106a5b5019
pcl::PointCloud<pcl::PointXYZ> filter::plane_fit(pcl::PointCloud<pcl::PointXYZ> cloud) {
}
pcl::PointCloud<pcl::PointXYZ> filter::box_range(pcl::PointCloud<pcl::PointXYZ> cloud) {
<<<<<<< HEAD
=======

>>>>>>> dc9189feef3b252f4d36b2efa99014106a5b5019
}
pcl::PointCloud<pcl::PointXYZ> filter::circle_range(pcl::PointCloud<pcl::PointXYZ> cloud) {
}
pcl::PointCloud<pcl::PointXYZ> filter::fov_range(pcl::PointCloud<pcl::PointXYZ> cloud) {
  double fov = 180;
  double minradius = 0;
  double maxradius = 30;
  double rad_to_deg = 180 / M_PI;
<<<<<<< HEAD
  pcl::PointCloud<pcl::PointXYZ> cloud_radius_filtered;
  outrem.filter(*cloud_radius_filtered);
=======
>>>>>>> dc9189feef3b252f4d36b2efa99014106a5b5019
}
pcl::PointCloud<pcl::PointXYZ> filter::random_subset(pcl::PointCloud<pcl::PointXYZ> cloud) {
}
<<<<<<< HEAD
=======

>>>>>>> dc9189feef3b252f4d36b2efa99014106a5b5019
pcl::PointCloud<pcl::PointXYZ> filter::covered_centroid(pcl::PointCloud<pcl::PointXYZ> cloud) {
}
pcl::PointCloud<pcl::PointXYZ> filter::voxel_downsample(pcl::PointCloud<pcl::PointXYZ> cloud) {
<<<<<<< HEAD
=======

>>>>>>> dc9189feef3b252f4d36b2efa99014106a5b5019
}