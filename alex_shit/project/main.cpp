#include "GraceAndConrad.hpp"
#include "external/csv-parser-2.3.0/single_include/csv.hpp"
#include <cuda_runtime.h>
#include <iostream>
#include "Point.hpp"

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

    float alpha = 0.1f;
    int num_bins = 250;
    float height_threshold = 0.5f;


    auto start = std::chrono::high_resolution_clock::now();

    auto result = GraceAndConrad(points, alpha, num_bins, height_threshold);

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

    // Print the time taken
    std::cout << "GraceAndConrad execution time: " << duration.count() << " ms" << std::endl;

    // Print the number of results
    std::cout << "Number of filtered points: " << result.size() << std::endl;

    // std::cout << "Filtered points:" << std::endl;
    // for (const auto &p : result) {
    //     std::cout << p.x << "," << p.y << "," << p.z << std::endl;
    // }

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
    for (const auto &p : result) {
        outfile << p.x << "," << p.y << "," << p.z << "\n";
    }

    outfile.close();
    std::cout << "Filtered points written to " << output_csv_file << std::endl;

    return 0;
}