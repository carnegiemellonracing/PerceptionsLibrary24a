#ifndef DBSCAN_H
#define DBSCAN_H

#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include "Point.hpp"

std::vector<Point> runDBSCAN(const std::vector<Point>& h_points, float eps, int min_samples);
#endif