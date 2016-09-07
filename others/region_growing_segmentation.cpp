#include <iostream>
#include <vector>
#include <cmath>
#include <pcl/pcl_base.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>

void
downsample (pcl::PointCloud<pcl::PointXYZ>::Ptr &points, float leaf_size,
            pcl::PointCloud<pcl::PointXYZ>::Ptr &downsampled_out)
{
  pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
  vox_grid.setLeafSize (leaf_size, leaf_size, leaf_size);
  vox_grid.setInputCloud (points);
  vox_grid.filter (*downsampled_out);
}


int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if ( pcl::io::loadPCDFile <pcl::PointXYZ> (argv[1], *cloud) == -1)
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZ>);
  const float voxel_grid_leaf_size = 4.0;
  downsample(cloud, voxel_grid_leaf_size,cloud_downsampled);
  std::cerr<<"number of cloud_downsampled : "<< cloud_downsampled->points.size() << " data points" << endl;
  pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud_downsampled);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);

  pcl::PointCloud<pcl::PointNormal>::Ptr cloudnormals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields(*cloud_downsampled, *normals, *cloudnormals);

  pcl::RegionGrowing<pcl::PointNormal, pcl::Normal> reg;
  reg.setMinClusterSize (50);
  reg.setMaxClusterSize (1000000);
  reg.setSearchMethod (pcl::search::Search<pcl::PointNormal>::Ptr (new pcl::search::KdTree<pcl::PointNormal>));
  reg.setNumberOfNeighbours (30);
  reg.setInputCloud (cloudnormals);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (6.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold (2.0);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;

/*
  int counter = 0;
  while (counter < clusters[0].indices.size ())
  {
    std::cout << clusters[0].indices[counter] << ", ";
    counter++;
    if (counter % 10 == 0)
      std::cout << std::endl;
  }
  std::cout << std::endl;
*/

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  int count =0;   
  pcl::PointCloud<pcl::PointNormal>::Ptr clustered_cloud (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr wall (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr floor (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr chaque_point (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointIndices::Ptr inliers_wall (new pcl::PointIndices);
  pcl::PointIndices::Ptr inliers_floor (new pcl::PointIndices);
  pcl::PointIndices::Ptr for_ptr (new pcl::PointIndices);

  for (std::vector<pcl::PointIndices>::iterator it = clusters.begin (); it != clusters.end (); ++it) 
  {
    *for_ptr = *it;
    pcl::ExtractIndices<pcl::PointNormal> ifilter (true);
    ifilter.setInputCloud (cloudnormals);
    ifilter.setIndices (for_ptr);
    ifilter.setNegative (false);
    ifilter.filter (*chaque_point);
    
    pcl::visualization::PCLVisualizer viewer ("results");
    viewer.addPointCloud(chaque_point, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> (chaque_point, 255.0, 255.0, 255.0), "clustered points");
    viewer.spin();

    pcl::PointCloud<pcl::PointNormal>::Ptr points_clustered (new pcl::PointCloud<pcl::PointNormal>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    { 
        
      points_clustered->points.push_back (cloudnormals->points[*pit]);
    }
    std::cout<< "size :"<<points_clustered->size()<<std::endl;
    if (points_clustered->size() > clusters[0].indices.size ())
    {
      points_clustered->height = 1;
      points_clustered->width = points_clustered->size();
/*
      std::stringstream name_point;
      name_point<<"lalala_group"<< count<<".pcd";
      pcl::PCDWriter writer;
      pcl::io::savePCDFileASCII(name_point.str(), *points_clustered);
      count ++;
*/
      pcl::PointNormal min_;
      pcl::PointNormal max_;
      pcl::getMinMax3D (*points_clustered, min_, max_ );
      int num_floor_a = 0;
      int num_floor_b = 0;
      int num_wall =0;
      int num_only_one_wall_a = 0;
      int num_only_one_wall_b = 0;

      for (size_t j = 0; j < points_clustered->size(); j++)
      {

        const Eigen::Vector3f initial_normal (points_clustered->points[j].normal_x,points_clustered->points[j].normal_y,points_clustered->points[j].normal_z);
        const Eigen::Vector3f floor_normal (1.0,-1.0, 0.0);
        const Eigen::Vector3f wall_normal (0.0, 0.0, -1.0);
        float dot_product_f = fabsf (floor_normal.dot (initial_normal));
        float dot_product_w = fabsf (wall_normal.dot (initial_normal));
        if (max_.y - points_clustered->points[j].y < 30)  num_floor_a ++;
        if (dot_product_f > 0.9)                          num_floor_b ++;
        if (dot_product_f > -0.5 && dot_product_f < 0.5)  num_wall ++ ;
        if (max_.z - points_clustered->points[j].z < 30)  num_only_one_wall_a ++;
        if (dot_product_w > 0.9)                          num_only_one_wall_b ++;
      }


      if (num_floor_b > int ( points_clustered->size() /1.2 )) //num_floor_a > (points_clustered->size() /2)
      {
        std::cout << " this is the floor!! "<<std::endl;
        *floor = * points_clustered;
        *inliers_floor = *it;
      }
      if (num_wall > (points_clustered->size() /1.5))
      {
        std::cout << " this is the wall!! "<<std::endl;
        *wall = * points_clustered;
        *inliers_wall = *it;
        if ( num_only_one_wall_b > (points_clustered->size() /1.2))
        {
          std::cout << " this is the only wall in front!! "<<std::endl;
        }
      }
    }
  }
/*
//  pcl::IndicesConstPtr indices_rem;
  pcl::PointIndices::Ptr indices_out (new pcl::PointIndices);
  pcl::PointCloud<pcl::PointNormal>::Ptr clustered_out (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr clustered_out_ (new pcl::PointCloud<pcl::PointNormal>);
  pcl::ExtractIndices<pcl::PointNormal> eifilter (false); // Initializing with true will allow us to extract the removed indices
  eifilter.setInputCloud (cloudnormals);
  eifilter.setIndices (inliers_floor);
  eifilter.getRemovedIndices (*indices_out);
  eifilter.setNegative (true);
  eifilter.filterDirectly (cloudnormals);
  std::cerr<<"clustered_out : There are "<< cloudnormals->points.size() << " data points" << endl;


  pcl::ExtractIndices<pcl::PointNormal> ifilter (true);
  ifilter.setInputCloud (cloudnormals);
  ifilter.setIndices (inliers_wall);
  ifilter.getRemovedIndices (*indices_out);
  ifilter.setNegative (false);
  ifilter.filter (*clustered_out_);

  std::cerr<<"clustered_out_ : There are "<< clustered_out_->points.size() << " data points" << endl;
*/
/*
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  pcl::visualization::PCLVisualizer viewer ("results");
  viewer.addPointCloud(colored_cloud, "colored points");
  viewer.spin();

  pcl::visualization::PCLVisualizer viz ("Cluster");
  viz.addPointCloud(clustered_out_, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> (clustered_out_, 255.0, 255.0, 255.0), "clustered points");
//  viz.addPointCloud(cloudnormals,pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> (cloudnormals, 255.0, 255.0, 255.0), "clustered points");
//  viz.addPointCloud (wall, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> (wall, 255.0, 255.0, 0.0), "wall");
//  viz.addPointCloud (floor, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> (floor, 255.0, 0.0, 0.0), "floor");
  viz.spin ();
*/
/*
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("region_model.pcd", *clustered_cloud, false);
  std::cerr<<"Saved "<< colored_cloud->points.size() << " data points" << endl;
*/
/*
  pcl::visualization::PCLVisualizer viewer ("Cluster viewer");
  viewer.addPointCloud(clustered_cloud,"clustered points");
  viewer.spin ();
*/
  return (0);
}

