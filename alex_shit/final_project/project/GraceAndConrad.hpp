#ifndef GRACEANDCONRAD_H
#define GRACEANDCONRAD_H

#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include "Point.hpp"

thrust::host_vector<Point> GraceAndConrad(
    const thrust::host_vector<Point>& points,
    float alpha,
    int num_bins,
    float height_threshold);
#endif