#include "filter.h"
#include <cmath>



pcl::PointCloud<pcl::PointXYZ> trim_cloud(pcl::PointCloud<pcl::PointXYZ> cloud) {

}

pcl::PointCloud<pcl::PointXYZ> remove_ground(pcl::PointCloud<pcl::PointXYZ> cloud) {

}

pcl::PointCloud<pcl::PointXYZ> GraceAndConrad(pcl::PointCloud<pcl::PointXYZ> cloud,
                  int points_ground, int alpha, int num_bins, int height_threshold) {
    
    std::vector<float> angles;
    for (const auto& point : cloud) {
        // Use atan2(y, x) to calculate the angle in radians
        float angle = std::atan2(cloud.y, cloud.x);

        // Store the angle in the vector
        angles.push_back(angle);
    }


}
                  
pcl::PointCloud<pcl::PointXYZ> section_pointcloud(pcl::PointCloud<pcl::PointXYZ> cloud,
                  int boxdim_x, int boxdim_y) {

}

pcl::PointCloud<pcl::PointXYZ> fit_sections(pcl::PointCloud<pcl::PointXYZ> cloud) {

}

pcl::PointCloud<pcl::PointXYZ> box_range(pcl::PointCloud<pcl::PointXYZ> cloud) {

}

pcl::PointCloud<pcl::PointXYZ> circle_range(pcl::PointCloud<pcl::PointXYZ> cloud) {

}

pcl::PointCloud<pcl::PointXYZ> fov_range(pcl::PointCloud<pcl::PointXYZ> cloud) {

}

pcl::PointCloud<pcl::PointXYZ> voxel_downsample(pcl::PointCloud<pcl::PointXYZ> cloud) {
  
}