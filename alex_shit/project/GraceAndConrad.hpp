#ifndef GRACEANDCONRAD_H
#define GRACEANDCONRAD_H

#include <thrust/host_vector.h>
#include <thrust/device_vector.h>

thrust::host_vector<float3> GraceAndConrad(
    const thrust::host_vector<float3> &points,
    float alpha,
    int num_bins,
    float height_threshold);

#endif