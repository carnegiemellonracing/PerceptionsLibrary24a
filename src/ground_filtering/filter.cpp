#include <iostream>
#include <fstream>
#include <numeric>
#include <algorithm>
#include <vector>
#include <chrono>
#include <bits/stdc++.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define _USE_MATH_DEFINES
#include <cmath>

//#define DEBUG
//#define SUBTIMING

using namespace std;
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;
using namespace pcl;

/*  Old data structure: no longer needed
typedef struct point {
  double x;
  double y;
  double z;
} point_t;
*/

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
radial_t point2radial(PointXYZ pt) {
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
PointXYZ radial2point(radial_t rd) {
  PointXYZ pt;
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
  radial_t rd = bin[i];
  if (rd.z < mini.z) {
    mini = rd;
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
 * @return A point cloud of ground-filtered points
 */
PointCloud<PointXYZ> GraceAndConrad(PointCloud<PointXYZ> cloud, double alpha, 
                                    int num_bins, double height_threshold) {
  #ifdef SUBTIMING
  auto t1 = high_resolution_clock::now();
  auto t2 = high_resolution_clock::now();
  #endif

  const double angle_min = -0.5 * M_PI;
  const double angle_max = 0.5 * M_PI;
  const double radius_max = 40;
  int num_segs = static_cast<int>((angle_max - angle_min) / alpha);
  vector<vector<vector<radial_t>>> segments(num_segs, vector<vector<radial_t>>(num_bins));
  PointCloud<PointXYZ> output;

  #ifdef SUBTIMING
  t1 = high_resolution_clock::now();
  #endif

  // Parse all points from XYZ to radial,Z and separate into bins
  int csize = cloud.points.size();
  for (int i = 0; i < csize; i++) {
  radial_t rd = point2radial(cloud.points[i]);
  if (rd.radius < radius_max) {
    int seg_index = static_cast<int>(rd.angle / alpha) + num_segs / 2 - (rd.angle < 0);
    int bin_index = static_cast<int>(rd.radius / (radius_max / num_bins));
    if (seg_index < 0)
    seg_index = 0;
    if (seg_index >= num_segs)
    seg_index = num_segs - 1;
    segments[seg_index][bin_index].push_back(rd);   // This line is doubling the execution time of sector 1
  }
  }

  #ifdef SUBTIMING 
  t2 = high_resolution_clock::now();
  printf("Sector 1: %f ms\n", duration<double, std::milli>(t2 - t1).count());
  #endif

  #ifdef DEBUG
  // Test code
  int seg_size =  segments.size();
  for (int i = 0; i < seg_size; i++) {
  printf("Segment (%d): ", i);
  if (i < 10) printf(" ");
  for (int j = 0; j < num_bins; j++) {
    printf("| %ld ", segments[i][j].size());
  }
  printf("|\n");
  }
  #endif

  // Grace and Conrad Algorithm
  for (int seg = 0; seg < num_segs; seg++) {
  // Extract minimum points in each bin

  #ifdef SUBTIMING
  t1 = high_resolution_clock::now();
  #endif

  vector<double> minis_rad = {};
  vector<double> minis_z = {};
  for (int bin = 0; bin < num_bins; bin++) {
    radial_t mini = min_height(segments[seg][bin]);
    if (mini.radius != -100) {
    minis_rad.push_back(mini.radius);
    minis_z.push_back(mini.z);
    }
  }

  #ifdef SUBTIMING 
  t2 = high_resolution_clock::now();
  printf("Sector 2a: %f ms\n", duration<double, std::milli>(t2 - t1).count());
  #endif

  #ifdef SUBTIMING
  t1 = high_resolution_clock::now();
  #endif

  // Performing linear regression
  double sum_rad = 0;
  double sum_rad2 = 0;
  double sum_z = 0;
  double sum_radz = 0;
  int n = minis_rad.size();
  for (int i = 0; i < n; i++) {
    double rad = minis_rad[i];
    double z = minis_z[i];
    sum_rad += rad;
    sum_rad2 += rad * rad;
    sum_z += z;
    sum_radz += rad * z;
  }
  
  // Calculating slope and intercept
  double slope = 0;
  double intercept = sum_z;
  if (n > 1) {
    slope = (n * sum_radz - sum_rad * sum_z) / (n * sum_rad2 - sum_rad * sum_rad);
    intercept = (sum_z - slope * sum_rad) / n;
  }
  #ifdef SUBTIMING 
  t2 = high_resolution_clock::now();
  printf("Sector 2b: %f ms\n", duration<double, std::milli>(t2 - t1).count());
  #endif

  #ifdef SUBTIMING
  t1 = high_resolution_clock::now();
  #endif
  // Convert all correct points to xyz and push to output vector
  for (int bin = 0; bin < num_bins; bin++) {
    for (int j = segments[seg][bin].size() - 1; j >= 0; j--) {
    radial_t pt = segments[seg][bin][j];
    double cutoff = slope * pt.radius + intercept + height_threshold;
    if (pt.z > cutoff) {
      output.points.push_back(radial2point(pt));
    }
    }
  }
  #ifdef SUBTIMING 
  t2 = high_resolution_clock::now();
  printf("Sector 2c: %f ms\n", duration<double, std::milli>(t2 - t1).count());
  #endif
  }

  return output;
}

// Test code
int main() {

  double time_sum = 0;
  double points_sum = 0;

  for (int i = 50; i < 243; i++) {
  PointCloud<PointXYZ> cloud;
  // Create a text string, which is used to output the text file
  string buf;

  // Read from the text file
  printf("\nRunning GAC on dataset %d\n", i);
  string file_name = "../point_clouds/point_cloud_" + to_string(i) + ".csv";
  ifstream point_data(file_name);
  file_name = "../point_clouds/parsed_point_cloud_" + to_string(i) + ".csv";
  ofstream write_to(file_name);

  if (point_data.is_open() ) {
    while (point_data.good() ) {
    point_data >> buf;

    vector<string> v;
    stringstream ss(buf);
  
    while (ss.good()) {
      string substr;
      getline(ss, substr, ',');
      v.push_back(substr);
    }
  
    float x = stof(v.back());
    v.pop_back();
    float y = stof(v.back());
    v.pop_back();
    float z = stof(v.back());
    v.pop_back();
    // +y is forward
    // +-z is left/right
    // +x is up
    cloud.push_back({y, z, x});
    }
  }

  // Close the file
  point_data.close();

  auto t1 = high_resolution_clock::now();
  PointCloud<PointXYZ> parsed_cloud = GraceAndConrad(cloud, 0.1, 10, 0.13);
  auto t2 = high_resolution_clock::now();
  double exe_time = duration<double, std::milli>(t2 - t1).count();
  time_sum += exe_time;
  points_sum += cloud.size();

  printf("Input cloud size: %ld\n", cloud.size());
  printf("Output cloud size: %ld\n", parsed_cloud.size());
  printf("Execution time: %f ms\n", exe_time);

  int sz = parsed_cloud.size();
  write_to << "x,y,z\n";
  for (int i = 0; i < sz; i++) {
    write_to << to_string(parsed_cloud.points[i].z) + "," + to_string(parsed_cloud.points[i].x) + "," + to_string(parsed_cloud.points[i].y) + "\n";
  }
  write_to.close();

  }
  printf("\nAverage execution speed: %0.0f points/ms\n", points_sum/time_sum);
}
 