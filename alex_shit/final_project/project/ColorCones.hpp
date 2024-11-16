#ifndef INTENSITYCLUSTER_H
#define INTENSITYCLUSTER_H

#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include "Point.hpp"

std::vector<std::pair<Point, std::string>> ColorCones(const std::vector<Point>& points);
#endif