#include <iostream>
#include <vector>

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
  if (rd.angle < 0) {
    rd.angle += 2 * M_PI;
  }
  rd.radius = std::sqrt(pt.x * pt.x + pt.y * pt.y);
  rd.z = pt.z;
  return rd;
}

point_t radial2point(radial_t rd) {
  point_t pt;
  return pt;
}

/**
 * Function implementing the GraceAndConrad algorithm
 * @param cloud: The vector of rectangular points to parse
 * @param alpha: 
 * @param num_bins: 
 * @param height_threshold:
 */
std::vector<point_t> GAC(std::vector<point_t> cloud, double alpha, 
                         int num_bins, double height_threshold) {
    
  // Convert all points to radials and find min/max angle
  std::vector<point_t> filtered_cloud = {};

  const double angle_min = 0;
  const double angle_max = 180;
  int num_segs = static_cast<int>((angle_max - angle_min) / alpha);
  std::vector<radial_t> segments[num_segs]; 

  for (int i = 0; i < cloud.size(); i++) {
    radial_t rd = point2radial(cloud[i]);
    segments[static_cast<int>(rd.angle / alpha)].push_back(rd);
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
  return NULL;
}

int main() {
    std::vector<point_t> cloud;
    clo
    d.push_back;
}