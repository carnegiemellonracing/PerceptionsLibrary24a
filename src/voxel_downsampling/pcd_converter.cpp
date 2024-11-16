#include <iostream>
#include <filesystem>
#include <string>
#include <vector>
#include "cnpy.h" // Include the npz reading library
#include <pcl/io/pcd_io.h> // Include PCL library for handling PCD files
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace fs = std::filesystem;

void processPCD(const std::string& pcdFilename) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    // Load the PCD file
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcdFilename, *cloud) == -1) {
        PCL_ERROR("Couldn't read file %s \n", pcdFilename.c_str());
        return;
    }

    std::cout << "Loaded " << cloud->width * cloud->height << " data points from " << pcdFilename << std::endl;

    // You can now process the point cloud data in the 'cloud' object
}

void extractPCDFromNPZ(const std::string& npzFilename, const std::string& outputDir) {
    cnpy::npz_t npzData = cnpy::npz_load(npzFilename);

    for (const auto& npzEntry : npzData) {
        const std::string& key = npzEntry.first;

        if (key.find(".pcd") != std::string::npos) { // Check if the entry is a PCD file
            std::string pcdData(reinterpret_cast<char*>(npzEntry.second.data_holder->data()), npzEntry.second.num_bytes());
            std::string outputPCDFile = outputDir + "/" + key;

            // Write PCD data to file
            std::ofstream outFile(outputPCDFile, std::ios::binary);
            outFile.write(pcdData.data(), pcdData.size());
            outFile.close();

            std::cout << "Extracted PCD file: " << outputPCDFile << std::endl;

            // Process the PCD file
            processPCD(outputPCDFile);
        }
    }
}

void iterateNPZFiles(const std::string& folderPath, const std::string& outputDir) {
    for (const auto& entry : fs::directory_iterator(folderPath)) {
        if (entry.is_regular_file() && entry.path().extension() == ".npz") {
            std::cout << "Processing file: " << entry.path() << std::endl;
            extractPCDFromNPZ(entry.path().string(), outputDir);
        }
    }
}

int main() {
    std::string inputFolder = "/Users/beeboss/Desktop/CMR/perceptions/PerceptionsLibrary24a/src/voxel_downsampling/tt-4-18-eleventh";
    std::string outputFolder = "/Users/beeboss/Desktop/CMR/perceptions/PerceptionsLibrary24a/src/voxel_downsampling/output_pcd";

    // Ensure output directory exists
    fs::create_directories(outputFolder);

    // Iterate and process .npz files
    iterateNPZFiles(inputFolder, outputFolder);

    return 0;
}