#include <iostream>
#include <fstream>
#include <numeric>
#include <algorithm>
#include <vector>
#include <chrono>
#include <bits/stdc++.h>

#define _USE_MATH_DEFINES
#include <cmath>

using namespace std;
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

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

// Test code
int main() {

  double sum = 0;

  for (int i = 50; i < 51; i++) {
    std::vector<point_t> cloud;
    // Create a text string, which is used to output the text file
    string buf;

    // Read from the text file
    printf("\nRunning GAC on dataset %d\n", i);
    string file_name = "point_clouds/point_cloud_" + to_string(i) + ".csv";
    ifstream point_data(file_name);

    if (point_data.is_open() ) {
      double xmin = 1000;
      double xmax = -1000;
      double ymin = 1000;
      double ymax = -1000;
      double zmin = 1000;
      double zmax = -1000;
      double xsum = 0;
      double ysum = 0;
      double zsum = 0;
      while (point_data.good() ) {
        point_data >> buf;

        vector<string> v;
        stringstream ss(buf);
    
        while (ss.good()) {
          string substr;
          getline(ss, substr, ',');
          v.push_back(substr);
        }
    
        double x = stod(v.back());
        v.pop_back();
        double y = stod(v.back());
        v.pop_back();
        double z = stod(v.back());
        v.pop_back();
        cloud.push_back({x, y, z});
        if (x < xmin) xmin = x;
        if (x > xmax) xmax = x;
        if (y < ymin) ymin = y;
        if (y > ymax) ymax = y;
        if (z < zmin) zmin = z;
        if (z > zmax) zmax = z;
        xsum += x;
        ysum += y;
        zsum += z;
      }
      printf("xmin = %f\n", xmin);
      printf("xmax = %f\n", xmax);
      printf("ymin = %f\n", ymin);
      printf("ymax = %f\n", ymax);
      printf("zmin = %f\n", zmin);
      printf("zmax = %f\n", zmax);
      printf("xavg = %f\n", xsum/cloud.size());
      printf("yavg = %f\n", ysum/cloud.size());
      printf("zavg = %f\n", zsum/cloud.size());
    }


    // Close the file
    point_data.close();
  }
}
 