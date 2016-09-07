#include <vector>
#include <iostream>
#include <Eigen/Core>
#include <pcl/common/time.h>
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"
#include <pcl/search/kdtree.h>
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/features/normal_3d.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/vfh.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/console/parse.h>

void
downsample (pcl::PointCloud<pcl::PointXYZ>::Ptr &points, float leaf_size,
            pcl::PointCloud<pcl::PointXYZ>::Ptr &downsampled_out)
{
  pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
  vox_grid.setLeafSize (leaf_size, leaf_size, leaf_size);
  vox_grid.setInputCloud (points);
  vox_grid.filter (*downsampled_out);
}

void
compute_surface_normals (pcl::PointCloud<pcl::PointXYZ>::Ptr &points, float normal_radius,
                         pcl::PointCloud<pcl::Normal>::Ptr &normals_out)
{
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;

  // Use a FLANN-based KdTree to perform neighborhood searches
  //norm_est.setSearchMethod (pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr (new pcl::KdTreeFLANN<pcl::PointXYZ>));
  norm_est.setSearchMethod (pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));


  // Specify the size of the local neighborhood to use when computing the surface normals
  norm_est.setRadiusSearch (normal_radius);

  // Set the input points
  norm_est.setInputCloud (points);

  // Estimate the surface normals and store the result in "normals_out"
  norm_est.compute (*normals_out);
}


void
compute_VFH_features (pcl::PointCloud<pcl::PointXYZ>::Ptr &points,
                      pcl::PointCloud<pcl::Normal>::Ptr &normals,
                      pcl::PointCloud<pcl::VFHSignature308>::Ptr &descriptors_out)
{
  // Create a PFHEstimation object
  pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh_est;

  // Set the input points and surface normals
  vfh_est.setInputCloud (points);
  vfh_est.setInputNormals(normals);
  vfh_est.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
  // Optionally, we can normalize the bins of the resulting histogram,
  // using the total number of points.
  vfh_est.setNormalizeBins(true);
  // Also, we can normalize the SDC with the maximum size found between
  // the centroid and any of the cluster's points.
  vfh_est.setNormalizeDistance(false);

  std::cout << "start compute VFH features descriptors" << std::endl;
  // Compute the features
  vfh_est.compute (*descriptors_out);
  std::cout << "Compute VFH descriptors successfully" << std::endl;
/*
  pcl::visualization::PCLPlotter plotter;
  // We need to set the size of the descriptor beforehand.
  plotter.addFeatureHistogram(*descriptors_out, 308); 
  plotter.plot();

  // Plotter object.
  pcl::visualization::PCLHistogramVisualizer viewer;
  // We need to set the size of the descriptor beforehand.
  viewer.addFeatureHistogram(*descriptors_out, 308);
  viewer.spin();
*/
}

                               		 
int main (int argc, char *argv[])
{

  for(int i=0; i< 143; i++){

    pcl::PointCloud<pcl::PointXYZ>::Ptr points1 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled1 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals1 (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptors_vfh1 (new pcl::PointCloud<pcl::VFHSignature308>);

    std::stringstream ss1, ss2;
    ss1 << "training_model_"<<i<<".pcd";
    pcl::io::loadPCDFile (ss1.str(), *points1);
    const float voxel_grid_leaf_size1 = 4.0;
    downsample (points1, voxel_grid_leaf_size1, downsampled1);
    const float normal_radius1 = 4.5;
    compute_surface_normals (downsampled1, normal_radius1, normals1);
    compute_VFH_features (downsampled1, normals1, descriptors_vfh1);
    ss2 << "training_VFH_features_"<<i<<".pcd";
    pcl::io::savePCDFileBinary(ss2.str(), *descriptors_vfh1 );

  }

  return (0);
}
