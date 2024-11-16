#include "GraceAndConrad.hpp"
#include "csv.hpp"
#include <iostream>

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <csv_file_path>" << std::endl;
        return 1;
    }

    std::string csv_file_path = argv[1];
    thrust::host_vector<float3> points;

    csv::CSVReader reader(csv_file_path);
    for (csv::CSVRow &row : reader) {
        float x = row["x"].get<float>();
        float y = row["y"].get<float>();
        float z = row["z"].get<float>();
        points.push_back(make_float3(x, y, z));
    }

    float alpha = 0.1f;
    int num_bins = 10;
    float height_threshold = 0.5f;

    auto result = GraceAndConrad(points, alpha, num_bins, height_threshold);

    std::cout << "Filtered points:" << std::endl;
    for (const auto &p : result) {
        std::cout << p.x << "," << p.y << "," << p.z << std::endl;
    }

    return 0;
}