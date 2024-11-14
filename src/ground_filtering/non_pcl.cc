#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

typedef struct point {
  double x;
  double y;
  double z;
} point_t;

typedef struct radial {
  double angle;
  double radius;
  double z;
} radial_t;

radial_t point2radial(point_t pt) {
  radial_t rd;
  rd.angle = std::atan2(pt.y, pt.x);
  rd.radius = std::sqrt(pt.x * pt.x + pt.y * pt.y);
  rd.z = pt.z;
  // printf("(%f,%f,%f)\n", rd.radius, rd.angle * 180 / M_PI, rd.z);
  return rd;
}

point_t radial2point(radial_t rd) {
  point_t pt;
  return pt;
}

/**
 * Function implementing the GraceAndConrad algorithm
 * @param cloud: The vector of rectangular points to parse
 * @param alpha: The size of each segment (radians)
 * @param num_bins: The number of bins per segment
 * @param height_threshold: Keep all points this distance above the best fit line
 * @return std::vector<point_t>
 */
void GAC(vector<point_t> cloud, double alpha, 
                         int num_bins, double height_threshold) {

  const double angle_min = -0.5 * M_PI;
  const double angle_max = 0.5 * M_PI;
  const double radius_max = 30;
  int num_segs = static_cast<int>((angle_max - angle_min) / alpha);
  vector<vector<vector<radial_t>>> segments(num_segs, vector<vector<radial_t>>(num_bins));

  // Parse all points from XYZ to radial,Z and separate into bins
  for (int i = 0; i < cloud.size(); i++) {
    radial_t rd = point2radial(cloud[i]);
    int seg_index = static_cast<int>(rd.angle / alpha) + num_segs / 2 - (rd.angle < 0);
    int bin_index = static_cast<int>(rd.radius / (radius_max / num_bins));
    segments[seg_index][bin_index].push_back(rd);
  }

  // Test code
  for (int i = 0; i < segments.size(); i++) {
    for (int j = 0; j < segments[i].size(); j++) {
      printf("Segbin (%d,%d):\n", i, j);
      for (int k = 0; k < segments[i][j].size(); k++) {
        printf("(%f,%f,%f)\n", segments[i][j][k].radius, segments[i][j][k].angle * 180 / M_PI, segments[i][j][k].z);
      }
    }
  }

  // TODO: Begin GraceAndConrad algorithm
  /*
  // Process each segment
  for (int seg_idx = 0; seg_idx < M; ++seg_idx) {
      std::vector<std::pair<double, double>> Bines;
      std::vector<double> min_zs;
      for (int range_idx = 0; range_idx < num_bins; ++range_idx) {
          int bin_idx = seg_idx * num_bins + range_idx;
          // Get points in the current bin
          std::vector<int> idxs;
          for (size_t i = 0; i < grid_cell_indices.size(); ++i) {
              if (grid_cell_indices[i] == bin_idx) {
                  idxs.push_back(i);
              }
          }
          if (!idxs.empty()) {
              // Find the minimum Z in the bin
              double min_z = cloud[idxs[0]].z;
              int min_idx = idxs[0];
              for (int idx : idxs) {
                  if (cloud[idx].z < min_z) {
                      min_z = cloud[idx].z;
                      min_idx = idx;
                  }
              }
              double range = ranges[min_idx];
              Bines.emplace_back(range, min_z);
              min_zs.push_back(min_z);
          }
      }
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
          // Filter points in the segment
          for (size_t i = 0; i < cloud.size(); ++i) {
              if (segments[i] == seg_idx) {
                  double expected_z = slope * ranges[i] + intercept;
                  if (cloud[i].z > expected_z + height_threshold) {
                      filtered_cloud.push_back(cloud[i]);
                  }
              }
          }
      }
  }
  // filtered_cloud.width = filtered_cloud.size();
  // filtered_cloud.height = 1;
  // filtered_cloud.is_dense = true;
  */
  return;
}

// Test code
int main() {
  std::vector<point_t> cloud;
  cloud.push_back({10, 5, 0});
  cloud.push_back({5, -20, 0});
  GAC(cloud, M_PI / 4, 2, 10);
}


  
  /*
  // Map angles to segments
  std::vector<int> segments(angles.size());

  for (size_t i = 0; i < angles.size(); ++i) {
      segments[i] = static_cast<int>((angles[i] - angle_min) / alpha);
  }
  
  // Create range bins
  double rmin = *std::min_element(ranges.begin(), ranges.end());
  double rmax = *std::max_element(ranges.begin(), ranges.end());
  double bin_size = (rmax - rmin) / num_bins;
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
  */