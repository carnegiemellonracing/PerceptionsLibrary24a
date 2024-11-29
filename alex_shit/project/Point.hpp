// Point.hpp
#ifndef POINT_HPP
#define POINT_HPP

struct Point {
    float x, y, z;

    // Default constructor
    __host__ __device__ Point() : x(0), y(0), z(0) {}

    // Parameterized constructor
    __host__ __device__ Point(float x_val, float y_val, float z_val)
        : x(x_val), y(y_val), z(z_val) {}
};;

#endif // POINT_HPP