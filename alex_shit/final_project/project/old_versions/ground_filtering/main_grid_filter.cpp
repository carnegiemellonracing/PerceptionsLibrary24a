#include "GraceAndConrad.hpp"
#include "external/csv-parser-2.3.0/single_include/csv.hpp"
#include <cuda_runtime.h>
#include <iostream>
#include "Point.hpp"



#include <algorithm>
#include <vector>
#include <limits>

std::vector<Point> filterGroundPoints(
    const std::vector<Point>& h_points,
    float cell_size,
    float height_threshold) {

    // Compute bounding box of the point cloud
    float x_min = std::numeric_limits<float>::max();
    float x_max = std::numeric_limits<float>::lowest();
    float y_min = std::numeric_limits<float>::max();
    float y_max = std::numeric_limits<float>::lowest();

    for (const auto& point : h_points) {
        x_min = std::min(x_min, point.x);
        x_max = std::max(x_max, point.x);
        y_min = std::min(y_min, point.y);
        y_max = std::max(y_max, point.y);
    }

    // Calculate grid dimensions
    int grid_width = static_cast<int>((x_max - x_min) / cell_size) + 1;
    int grid_height = static_cast<int>((y_max - y_min) / cell_size) + 1;

    // Print bounding box and grid dimensions for debugging
    std::cout << "Bounding Box: (" << x_min << ", " << y_min << ") to (" << x_max << ", " << y_max << ")\n";
    std::cout << "Grid Dimensions: " << grid_width << " x " << grid_height << "\n";

    // Call the CUDA ground filtering function
    return filterGroundPointsCUDA(h_points, x_min, x_max, y_min, y_max, grid_width, grid_height);
}



int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <csv_file_path>" << std::endl;
        return 1;
    }

    std::string csv_file_path = argv[1];
    thrust::host_vector<Point> points;

    csv::CSVReader reader(csv_file_path);
    for (csv::CSVRow &row : reader) {
        float x = row["x"].get<float>();
        float y = row["y"].get<float>();
        float z = row["z"].get<float>();
        points.push_back(Point(x, y, z));
    }


    // Convert thrust::host_vector to std::vector for compatibility
    std::vector<Point> h_points(points.begin(), points.end());


    // Parameters
    float cell_size = 5.0f;          // Size of each grid cell in meters
    float height_threshold = 0.2f;  // Height threshold for ground points

    
    // Measure execution time
    auto start = std::chrono::high_resolution_clock::now();

    // Filter ground points
    std::vector<Point> ground_points = filterGroundPoints(h_points, cell_size, height_threshold);

    // Output results
    std::cout << "Number of non-ground points: " << ground_points.size() << "\n";
    // for (const auto& point : ground_points) {
    //     std::cout << "(" << point.x << ", " << point.y << ", " << point.z << ")\n";
    // }

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

    // Print the time taken
    std::cout << "filterGroundPointsCUDA execution time: " << duration.count() << " ms" << std::endl;


    // Output CSV file
    std::string output_csv_file = argv[2];
    std::ofstream outfile(output_csv_file);

    if (!outfile.is_open()) {
        std::cerr << "Failed to open output CSV file: " << output_csv_file << std::endl;
        return 1;
    }

    // Write CSV header
    outfile << "x,y,z\n";

    // Write filtered points to the output CSV file
    for (const auto &p : ground_points) {
        outfile << p.x << "," << p.y << "," << p.z << "\n";
    }

    outfile.close();
    std::cout << "Filtered points written to " << output_csv_file << std::endl;

    return 0;
}