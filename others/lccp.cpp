// Stdlib
#include <stdlib.h>
#include <cmath>
#include <limits.h>
#include <iterator>
#include <vector>
#include <boost/format.hpp>
#include <stdint.h>

// PCL input/output
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

//PCL other
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/features/normal_3d_omp.h>


// The segmentation class this example is for
#include <pcl/segmentation/lccp_segmentation.h>

typedef pcl::LCCPSegmentation<pcl::PointXYZ>::SupervoxelAdjacencyList SuperVoxelAdjacencyList;

void
compute_surface_normals (pcl::PointCloud<pcl::PointXYZ>::Ptr &points, float normal_radius,
                         pcl::PointCloud<pcl::Normal>::Ptr &normals_out)
{
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> norm_est;

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

int
main (int argc, char** argv)
{
  // Fetch point cloud filename in arguments | Works with PCD and PLY files
  std::vector<int> filenames;
  bool file_is_pcd = false;

  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");

  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ> ());

  if (pcl::io::loadPCDFile (argv[filenames[0]], *input_cloud_ptr) < 0)
  {
      std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
      return -1;
  }

  std::cout<< "NUmber of input points" << input_cloud_ptr->size () << std::endl;

  /// -----------------------------------|  Preparations  |-----------------------------------

  /// Create variables needed for preparations
  std::string outputname ("");
  pcl::PointCloud<pcl::Normal>::Ptr input_normals_ptr (new pcl::PointCloud<pcl::Normal>);
  float normal_radius = 5.0 ;
  compute_surface_normals (input_cloud_ptr, normal_radius, input_normals_ptr);


/// -----------------------------------|  Main Computation  |-----------------------------------

  ///  Default values of parameters before parsing
  // Supervoxel Stuff
  float voxel_resolution = 4.0f;
  float seed_resolution = 10.0f;
  float color_importance = 0.0f;
  float spatial_importance = 1.0f;
  float normal_importance = 4.0f;
  bool use_single_cam_transform = false;
  bool use_supervoxel_refinement = true;

  // LCCPSegmentation Stuff
  float concavity_tolerance_threshold = 10;
  float smoothness_threshold = 0.4;
  uint32_t min_segment_size = 4.0;
  bool use_extended_convexity = true;
  bool use_sanity_criterion = false;
  
  ///  Parse Arguments needed for computation
  //Supervoxel Stuff

  //  normals_scale = seed_resolution / 2.0;
  
  // Segmentation Stuff
  unsigned int k_factor = 0;
  if (use_extended_convexity)
    k_factor = 1;

  /// Preparation of Input: Supervoxel Oversegmentation

  pcl::SupervoxelClustering<pcl::PointXYZ> super (voxel_resolution, seed_resolution);
  super.setUseSingleCameraTransform (use_single_cam_transform);
  super.setInputCloud (input_cloud_ptr);

  super.setNormalCloud (input_normals_ptr);
  super.setColorImportance (color_importance);
  super.setSpatialImportance (spatial_importance);
  super.setNormalImportance (normal_importance);
  std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZ>::Ptr> supervoxel_clusters;

  PCL_INFO ("Extracting supervoxels\n");
  super.extract (supervoxel_clusters);

  if (use_supervoxel_refinement)
  {
    PCL_INFO ("Refining supervoxels\n");
    super.refineSupervoxels (2, supervoxel_clusters);
  }
  std::stringstream temp;
  temp << "  Nr. Supervoxels: " << supervoxel_clusters.size () << "\n";
  PCL_INFO (temp.str ().c_str ());

  PCL_INFO ("Getting supervoxel adjacency\n");
  std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
  super.getSupervoxelAdjacency (supervoxel_adjacency);

  /// Get the cloud of supervoxel centroid with normals and the colored cloud with supervoxel coloring (this is used for visulization)
  pcl::PointCloud<pcl::PointNormal>::Ptr sv_centroid_normal_cloud = pcl::SupervoxelClustering<pcl::PointXYZ>::makeSupervoxelNormalCloud (supervoxel_clusters);

  /// The Main Step: Perform LCCPSegmentation

  PCL_INFO ("Starting Segmentation\n");
  pcl::LCCPSegmentation<pcl::PointXYZ> lccp;
  lccp.setConcavityToleranceThreshold (concavity_tolerance_threshold);
  lccp.setSanityCheck (use_sanity_criterion);
  lccp.setSmoothnessCheck (true, voxel_resolution, seed_resolution, smoothness_threshold);
  lccp.setKFactor (k_factor);
  lccp.setInputSupervoxels (supervoxel_clusters, supervoxel_adjacency);
  lccp.setMinSegmentSize (min_segment_size);
  lccp.segment ();

  PCL_INFO ("Interpolation voxel cloud -> input cloud and relabeling\n");
  pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = super.getLabeledCloud ();
  pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud = sv_labeled_cloud->makeShared ();
  lccp.relabelCloud (*lccp_labeled_cloud);
  SuperVoxelAdjacencyList sv_adjacency_list;
  lccp.getSVAdjacencyList (sv_adjacency_list);  // Needed for visualization
  std::cout<< "number of lccp points : " << lccp_labeled_cloud->size () << std::endl;


  // storing all the indices of the LCCP groups

  std::vector<uint32_t> index_group; 
  for (size_t i = 0; i < lccp_labeled_cloud->size (); ++i) 
  {
    std::cout<<"It's round "<< i << ": lable is "<< lccp_labeled_cloud-> at (i).label<< std::endl;
    if (i == 0)  index_group.push_back(lccp_labeled_cloud-> at (0).label);
    else 
    {
      bool flag = false;
      for (auto pit : index_group)
      {
        std::cout<< pit <<std::endl;
        if ( pit == lccp_labeled_cloud-> at (i).label)  flag = true;
      }

      if (!flag)  index_group.push_back(lccp_labeled_cloud-> at (i).label);
    }

  }


  std::cout << "start!!!!" << std::endl;

  for (auto pit : index_group)
  {
    std::cout<< pit << std::endl;
  }

  // storing every group of cloud points into the vector 

  //  std::vector<pcl::PointCloud<pcl::PointXYZ>> points_group;
  int count = 1;
  for (auto pit : index_group)
  {
    pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_ (new pcl::PointCloud<pcl::PointXYZL> ());
    for (size_t i = 0; i < lccp_labeled_cloud->size (); ++i)
    {
      if (lccp_labeled_cloud-> at (i).label == pit)
      {
        cloud_->push_back(lccp_labeled_cloud->at(i));
      }
    }
    if (cloud_->size()>10000)
    {
      std::stringstream name_point;
      name_point<<"cloud_group"<< count<<".pcd";
      pcl::io::savePCDFileASCII(name_point.str(), *cloud_);
      count ++;
    }
//    points_group.push_back(cloud_);
  }

/*
  size_t number_max =2000;
//
  int count =1;
  for (auto pit : points_group)
  {
    std::stringstream name_point;
    name_point<<"cloud_group"<< count;
    if (points_group.size() > number_max)
    {
      pcl::io::savePCDFileASCII(name_point.str(), pit);
    }
    count ++;
  }
*/
/*                       
typedef pcl::LCCPSegmentation<pcl::PointXYZ>::VertexIterator VertexIterator;
            

std::pair<VertexIterator, VertexIterator> vertex_iterator_range;
vertex_iterator_range = boost::vertices (sv_adjacency_list);

            
for (VertexIterator itr = vertex_iterator_range.first; itr != vertex_iterator_range.second; ++itr)
{
	const uint32_t sv_label = sv_adjacency_list[*itr];
	pcl::PointXYZL group_label = lccp_labeled_cloud->points[ sv_label ];
        std::cout << "sv_label: " << sv_label << "; group_label.label: " << group_label.label <<std::endl;
//	printf(“%d %d\n”, sv_label, group_label.label); // supervoxel label, segmentation group label

}

/*
  /// Creating Colored Clouds and Output
  if (lccp_labeled_cloud->size () == input_cloud_ptr->size ())
  {
    if (output_specified)
    {
      PCL_INFO ("Saving output\n");
      if (add_label_field)
      {
        if (pcl::getFieldIndex (input_pointcloud2, "label") >= 0)
          PCL_WARN ("Input cloud already has a label field. It will be overwritten by the lccp segmentation output.\n");
        pcl::PCLPointCloud2 output_label_cloud2, output_concat_cloud2;
        pcl::toPCLPointCloud2 (*lccp_labeled_cloud, output_label_cloud2);
        pcl::concatenateFields (input_pointcloud2, output_label_cloud2, output_concat_cloud2);
        pcl::io::savePCDFile (outputname + "_out.pcd", output_concat_cloud2, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), save_binary_pcd);
      }
      else
        pcl::io::savePCDFile (outputname + "_out.pcd", *lccp_labeled_cloud, save_binary_pcd);

      if (sv_output_specified)
      {
        pcl::io::savePCDFile (outputname + "_svcloud.pcd", *sv_centroid_normal_cloud, save_binary_pcd);
      }
    }
  }
  else
  {
    PCL_ERROR ("ERROR:: Sizes of input cloud and labeled supervoxel cloud do not match. No output is produced.\n");
  }
*/
  return (0);
}
