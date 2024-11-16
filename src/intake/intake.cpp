#include <iostream>
#include <filesystem>
#include <cnpy.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace fs = std::filesystem;

void convertNPZtoPCD(const std::string& input_folder, const std::string& output_folder) {
    for (const auto& entry : fs::directory_iterator(input_folder)) {
        if (entry.path().extension() == ".npz") {
            std::string input_file = entry.path().string();
            std::string output_file = output_folder + "/" + entry.path().stem().string() + ".pcd";

            // Load the .npz file
            cnpy::npz_t npz_data = cnpy::npz_load(input_file);
            auto point_data = npz_data["points"];  // Assuming the key is "points"
            if (point_data.shape.size() != 2 || point_data.shape[1] != 3) {
                std::cerr << "Invalid point cloud data in file: " << input_file << std::endl;
                continue;
            }

            size_t num_points = point_data.shape[0];
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
            cloud->width = static_cast<uint32_t>(num_points);
            cloud->height = 1;
            cloud->is_dense = false;
            cloud->points.resize(num_points);

            // Populate the PointCloud
            float* data = point_data.data<float>();
            for (size_t i = 0; i < num_points; ++i) {
                cloud->points[i].x = data[i * 3];
                cloud->points[i].y = data[i * 3 + 1];
                cloud->points[i].z = data[i * 3 + 2];
            }

            // Save to .pcd
            if (pcl::io::savePCDFileASCII(output_file, *cloud) == -1) {
                std::cerr << "Error saving file: " << output_file << std::endl;
            } else {
                std::cout << "Saved " << output_file << std::endl;
            }
        }
    }
}

int main() {
    std::string input_folder = "/Users/beeboss/Desktop/CMR/perceptions/PerceptionsLibrary24a/src/tt-4-18-eleventh";
    std::string output_folder = "/Users/beeboss/Desktop/CMR/perceptions/PerceptionsLibrary24a/src/intake/pcd_files";

    // Create output directory if it doesn't exist
    fs::create_directories(output_folder);

    convertNPZtoPCD(input_folder, output_folder);

    return 0;
}