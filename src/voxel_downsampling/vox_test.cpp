#include <iostream>
<<<<<<< Updated upstream
#include <filesystem>
=======
>>>>>>> Stashed changes
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

<<<<<<< Updated upstream



=======
>>>>>>> Stashed changes
int
main ()
{
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

  // Fill in the cloud data
  pcl::PCDReader reader;
<<<<<<< Updated upstream
  pcl::PCDWriter writer;
  // Replace the path below with the path where you saved your file
  reader.read ("/Users/beeboss/Desktop/CMR/perceptions/PerceptionsLibrary24a/src/voxel_downsampling/test_pc.pcd", *cloud);
  writer.write ("/Users/beeboss/Desktop/CMR/perceptions/PerceptionsLibrary24a/src/voxel_downsampling/test_pc.pcd", *cloud);
=======
  // Replace the path below with the path where you saved your file
  reader.read ("table_scene_lms400.pcd", *cloud); // Remember to download the file first!
>>>>>>> Stashed changes

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;

<<<<<<< Updated upstream
  
  writer.write ("/Users/beeboss/Desktop/CMR/perceptions/PerceptionsLibrary24a/src/voxel_downsampling/output_pcd/reduced_boi.pcd", *cloud_filtered, 
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

  return (0);
}
=======
  pcl::PCDWriter writer;
  writer.write ("table_scene_lms400_downsampled.pcd", *cloud_filtered, 
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

  return (0);
}
>>>>>>> Stashed changes
