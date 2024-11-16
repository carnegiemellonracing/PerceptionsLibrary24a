#include <iostream>
#include <numeric>
#include <algorithm>
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

/**
 * Converts (x,y,z) to (radius,ang,z), where ang is in radians
 * @param pt: The (x,y,z) point to convert
 * @return the converted point
 */
radial_t point2radial(point_t pt) {
  radial_t rd;
  rd.angle = std::atan2(pt.y, pt.x);
  rd.radius = std::sqrt(pt.x * pt.x + pt.y * pt.y);
  rd.z = pt.z;
  return rd;
}

/**
 * Converts (radius,ang,z) to (x,y,z), where ang is in radians
 * @param rd: The (radius,ang,z) point to convert
 * @return the converted point
 */
point_t radial2point(radial_t rd) {
  point_t pt;
  pt.x = rd.radius * cos(rd.angle);
  pt.y = rd.radius * sin(rd.angle);
  pt.z = rd.z;
  return pt;
}

/**
 * Gets the minimum point in a bin
 * @param bin: The bin to search
 * @return the point with the lowest z
 */
radial_t min_height(vector<radial_t> bin) {
  int size = bin.size();
  if (size == 0) {
    return {-100, -100, -100};
  }

  radial_t mini = bin[0];
  for (int i = 0; i < size; i++) {
    if (bin[i].z < mini.z) {
      mini = bin[i];
    }
  }
  return mini;
}

/**
 * Function implementing the GraceAndConrad algorithm
 * @param cloud: The input vector of rectangular points to parse
 * @param alpha: The size of each segment (radians)
 * @param num_bins: The number of bins per segment
 * @param height_threshold: Keep all points this distance above the best fit line
 * @return std::vector<point_t>
 */
std::vector<point_t> GAC(vector<point_t> cloud, double alpha, 
                         int num_bins, double height_threshold) {

  const double angle_min = -0.5 * M_PI;
  const double angle_max = 0.5 * M_PI;
  const double radius_max = 30;
  int num_segs = static_cast<int>((angle_max - angle_min) / alpha);
  vector<vector<vector<radial_t>>> segments(num_segs, vector<vector<radial_t>>(num_bins));
  vector<point_t> output;

  // Parse all points from XYZ to radial,Z and separate into bins
  for (int i = 0; i < cloud.size(); i++) {
    radial_t rd = point2radial(cloud[i]);
    int seg_index = static_cast<int>(rd.angle / alpha) + num_segs / 2 - (rd.angle < 0);
    int bin_index = static_cast<int>(rd.radius / (radius_max / num_bins));
    segments[seg_index][bin_index].push_back(rd);
  }

  #ifdef DEBUG
  // Test code
  for (int i = 0; i < segments.size(); i++) {
    for (int j = 0; j < segments[i].size(); j++) {
      printf("Segbin (%d,%d):\n", i, j);
      for (int k = 0; k < segments[i][j].size(); k++) {
        printf("(%f,%f,%f)\n", segments[i][j][k].radius, segments[i][j][k].angle * 180 / M_PI, segments[i][j][k].z);
      }
    }
  }
  #endif

  // Grace and Conrad Algorithm
  for (int seg = 0; seg < num_segs; seg++) {
    // Extract minimum points in each bin
    if (segments[seg].size() <= 1) continue;
    vector<double> minis_rad = {};
    vector<double> minis_z = {};
    for (int bin = 0; bin < num_bins; bin++) {
      radial_t mini = min_height(segments[seg][bin]);
      if (mini.radius != -100) {
        minis_rad.push_back(mini.radius);
        minis_z.push_back(mini.z);
      }
    }
    
    #ifdef DEBUG
    for (int i = 0; i < minis_rad.size(); i++) {
      int bin_index = static_cast<int>(minis_rad[i] / (radius_max / num_bins));
      printf("Seg %d: mini at (%f,%f) in bin %d\n", seg, minis_rad[i], minis_z[i], bin_index);
    }
    #endif

    // Performing linear regression
    double sum_rad = 0;
    double sum_rad2 = 0;
    double sum_z = 0;
    double sum_radz = 0;
    int n = minis_rad.size();
    for (int i = 0; i < n; i++) {
      sum_rad += minis_rad[i];
      sum_rad2 += minis_rad[i] * minis_rad[i];
      sum_z += minis_z[i];
      sum_radz += minis_rad[i] * minis_z[i];
    }
    
    // Calculating slope and intercept
    double slope = 0;
    double intercept = 0;
    if (n > 1) {
      slope = (n * sum_radz - sum_rad * sum_z) / (n * sum_rad2 - sum_rad * sum_rad);
      intercept = (sum_z - slope * sum_rad) / n;
    } else if (n == 1) {
      slope = 0;
      intercept = sum_z;
    }

    // Convert all correct points to xyz and push to output vector
    for (int bin = 0; bin < num_bins; bin++) {
      for (int j = segments[seg][bin].size() - 1; j >= 0; j--) {
        radial_t pt = segments[seg][bin][j];
        double cutoff = slope * pt.radius + intercept + height_threshold;
        if (pt.z > cutoff) {
          output.push_back(radial2point(pt));
        }
      }
    }
  }

  return output;
}

// Test code
int main() {
  std::vector<point_t> cloud;
  cloud.push_back({10, -5, 1});
  cloud.push_back({10, -5, 2});
  cloud.push_back({10, 5, 0});
  cloud.push_back({10, 5.1, 1});
  cloud.push_back({10, 5.2, 2});
  cloud.push_back({10, 5.1, 3});
  cloud.push_back({10, 5.2, 4});
  cloud.push_back({10, 5.1, 5});
  cloud.push_back({10, 5.2, 6});
  cloud.push_back({10, 5.1, 7});
  cloud.push_back({10, 5.2, 8});
  cloud.push_back({20, 10.1, 1});
  cloud.push_back({20, 10.1, 5});
  cloud.push_back({20, 10.2, 6});
  cloud.push_back({20, 10.1, 7});
  cloud.push_back({20, 10.2, 8});
  cloud.push_back({5, -20, 0});
  vector<point_t> parsed_cloud = GAC(cloud, M_PI / 4, 2, 3);

  for (int i = 0; i < parsed_cloud.size(); i++) {
    point_t pt = parsed_cloud[i];
    printf("(%f,%f,%f)\n", pt.x, pt.y, pt.z);
  }
}
 