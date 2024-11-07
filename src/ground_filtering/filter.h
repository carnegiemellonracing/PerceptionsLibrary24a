#pragma once

namespace filter {

  pcl::PointCloud<pcl::PointXYZ> cloud;

  pcl::PointCloud<pcl::PointXYZ> trim_cloud(pcl::PointCloud<pcl::PointXYZ> cloud);
  pcl::PointCloud<pcl::PointXYZ> remove_ground(pcl::PointCloud<pcl::PointXYZ> cloud);
  pcl::PointCloud<pcl::PointXYZ> GraceAndConrad(pcl::PointCloud<pcl::PointXYZ> cloud,
                    int points_ground, int alpha, int num_bins, int height_threshold);
  pcl::PointCloud<pcl::PointXYZ> section_pointcloud(pcl::PointCloud<pcl::PointXYZ> cloud,
                    int boxdim_x, int boxdim_y);
  pcl::PointCloud<pcl::PointXYZ> fit_sections(pcl::PointCloud<pcl::PointXYZ> cloud);
  pcl::PointCloud<pcl::PointXYZ> plane_fit(pcl::PointCloud<pcl::PointXYZ> cloud);
  pcl::PointCloud<pcl::PointXYZ> box_range(pcl::PointCloud<pcl::PointXYZ> cloud);
  pcl::PointCloud<pcl::PointXYZ> circle_range(pcl::PointCloud<pcl::PointXYZ> cloud);
  pcl::PointCloud<pcl::PointXYZ> fov_range(pcl::PointCloud<pcl::PointXYZ> cloud);
  pcl::PointCloud<pcl::PointXYZ> random_subset(pcl::PointCloud<pcl::PointXYZ> cloud);
  pcl::PointCloud<pcl::PointXYZ> covered_centroid(pcl::PointCloud<pcl::PointXYZ> cloud);
  pcl::PointCloud<pcl::PointXYZ> voxel_downsample(pcl::PointCloud<pcl::PointXYZ> cloud);

}