#include <iostream>
#include <cmath>

#include "math.h"
#include "filter.h"
#include <cmath>



pcl::PointCloud<pcl::PointXYZ> filter::trim_cloud(pcl::PointCloud<pcl::PointXYZ> cloud) {
  double max_height = 5;
  double height = -1;
  double r_min = 1.5;
  double r_max = 10;
  double ang_cut = M_PI / 2;
  double scaling = 0.015;

}

pcl::PointCloud<pcl::PointXYZ> filter::remove_ground(pcl::PointCloud<pcl::PointXYZ> cloud) {

}

pcl::PointCloud<pcl::PointXYZ> filter::GraceAndConrad(pcl::PointCloud<pcl::PointXYZ> cloud,
                  int points_ground, int alpha, int num_bins, int height_threshold) {
    
    std::vector<float> angles;
        std::vector<float> angles;

    for (const auto& point : cloud) {
        float angle = std::atan2(cloud.y, cloud.x);
        angles.push_back(angle);
    }
    for (const float& value : angles) {
        if (angles < 0)
            bangles.push_back(value + 2 * MP_PI);
        else
            bangles.push_back(angles);

    }
    







}
                  
pcl::PointCloud<pcl::PointXYZ> filter::section_pointcloud(pcl::PointCloud<pcl::PointXYZ> cloud,
                  int boxdim_x, int boxdim_y) {

}

pcl::PointCloud<pcl::PointXYZ> filter::fit_sections(pcl::PointCloud<pcl::PointXYZ> cloud) {

}

<<<<<<< HEAD
pcl::PointCloud<pcl::PointXYZ> box_range(pcl::PointCloud<pcl::PointXYZ> cloud) {
=======
pcl::PointCloud<pcl::PointXYZ> filter::plane_fit(pcl::PointCloud<pcl::PointXYZ> cloud) {

}

pcl::PointCloud<pcl::PointXYZ> filter::box_range(pcl::PointCloud<pcl::PointXYZ> cloud) {
>>>>>>> 0dffa9f1e334d97e76340832118fb31b3832f9c8

}

pcl::PointCloud<pcl::PointXYZ> filter::circle_range(pcl::PointCloud<pcl::PointXYZ> cloud) {

}

pcl::PointCloud<pcl::PointXYZ> filter::fov_range(pcl::PointCloud<pcl::PointXYZ> cloud) {
  double fov = 180;
  double minradius = 0;
  double maxradius = 30;
  double rad_to_deg = 180 / M_PI;
  pcl::PointCloud<pcl::PointXYZ> cloud_radius_filtered;

  outrem.filter(*cloud_radius_filtered);
}

pcl::PointCloud<pcl::PointXYZ> filter::random_subset(pcl::PointCloud<pcl::PointXYZ> cloud) {

}

<<<<<<< HEAD
pcl::PointCloud<pcl::PointXYZ> voxel_downsample(pcl::PointCloud<pcl::PointXYZ> cloud) {
  
=======
pcl::PointCloud<pcl::PointXYZ> filter::covered_centroid(pcl::PointCloud<pcl::PointXYZ> cloud) {

}

pcl::PointCloud<pcl::PointXYZ> filter::voxel_downsample(pcl::PointCloud<pcl::PointXYZ> cloud) {

>>>>>>> 0dffa9f1e334d97e76340832118fb31b3832f9c8
}