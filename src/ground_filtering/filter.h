#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include "math.h"

namespace filter {

  pcl::PointCloud<pcl::PointXYZ> cloud;

  /**
   * Trims a cloud of points to reduce to a point cloud of only cone points
   */
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

  /**
   * Removes all points outside of a fields of view range 
   * (assumes even fov on left and right side)
   * and limits points to within the radius on x-y plane
   * 
   * @param pointcloud: pointcloud to remove from
   * @param fov: degrees pointcloud of resulting fov should be (e.g. 180 returns everything with +y value)
   * @param radius: max distance resulting points in pointcloud should be
   * @returns filtered point cloud where all points outside of fov are removed
   */
  pcl::PointCloud<pcl::PointXYZ> fov_range(pcl::PointCloud<pcl::PointXYZ> cloud);


  pcl::PointCloud<pcl::PointXYZ> random_subset(pcl::PointCloud<pcl::PointXYZ> cloud);


  pcl::PointCloud<pcl::PointXYZ> covered_centroid(pcl::PointCloud<pcl::PointXYZ> cloud);


  pcl::PointCloud<pcl::PointXYZ> voxel_downsample(pcl::PointCloud<pcl::PointXYZ> cloud);



}