#include "GraceAndConrad.hpp"
#include "IntensityCluster.hpp"
#include "DBscan.hpp"
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
        float intensity = 0; // Default intensity value

        try {
            intensity = row["intensity"].get<float>();
        } catch (const std::exception &e) {
            // Intensity column is missing; default value of 0 will be used
        }

        points.push_back(Point(x, y, z, intensity));
    }

    float alpha = 0.1f;
    int num_bins = 100;
    float height_threshold = 0.05f;


    auto start = std::chrono::high_resolution_clock::now();

    std::cout << "Number of raw points: " << points.size() << std::endl;

    auto result = GraceAndConrad(points, alpha, num_bins, height_threshold);

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

    // Print the time taken
    std::cout << "GraceAndConrad execution time: " << duration.count() << " ms" << std::endl;

    // Print the number of results
    std::cout << "Number of filtered points: " << result.size() << std::endl;

    // Call the clustering algorithm
    
    
    
    

    // DBSCAN
    
    std::vector<Point> std_points(result.begin(), result.end());

    int min_samples = 2;
    float eps = 0.3f;
    auto cone_clusters = runDBSCAN(std_points, eps, min_samples); // Call your coloring function here
    std::cout << "Number of clusters: " << cone_clusters.size() << std::endl;



    // Cone Coloring


    // Call the coloring algorithm
    std::cout << "Running cone coloring algorithm..." << std::endl;
    std::vector<Point> new_points(cone_clusters.begin(), cone_clusters.end());


    auto cone_colors = ClusterCones(new_points);  // Call your coloring function here

    // Write results to CSV
    std::string output_csv_file = argv[2];
    std::ofstream outfile(output_csv_file);
    if (!outfile.is_open()) {
        std::cerr << "Failed to open output CSV file: " << output_csv_file << std::endl;
        return 1;
    }

    // Write CSV header
    outfile << "x,y,z,intensity,color\n";

    // Write colored points to the output CSV file
    for (size_t i = 0; i < cone_colors.size(); ++i) {
        const auto& pair = cone_colors[i];
        const auto& p = pair.first; // Access the Point
        const auto& color = pair.second; // Access the color
        outfile << p.x << "," << p.y << "," << p.z << "," << p.intensity << "," << color << "\n";
    }

    outfile.close();
    std::cout << "Colored points written to " << output_csv_file << std::endl;

    return 0;
}