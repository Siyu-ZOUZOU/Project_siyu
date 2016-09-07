#include <iostream>
#include <vector>
#include <cmath>
#include <pcl/common/time.h>
#include <pcl/common/common.h> 
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/vfh.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/ml/svm_wrapper.h>
#include <pcl/registration/icp.h>

bool flag_floor = false;
bool flag_ceil = false;
bool flag_wall = false;
bool flag_wall_le = false;
bool flag_wall_ri = false;
bool flag_wall_in = false;

pcl::PointXYZ min;
pcl::PointXYZ max;

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
compute_surface_normals (pcl::PointCloud<pcl::PointXYZ>::Ptr &points, int neighbour_point,
                         pcl::PointCloud<pcl::Normal>::Ptr &normals_out)
{
  pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (points);
  normal_estimator.setKSearch (neighbour_point);
  normal_estimator.compute (*normals_out);
}

void
ransac_plane ( pcl::PointCloud<pcl::PointNormal>::Ptr &cloudnormals,
               pcl::PointIndices::Ptr &inliers,
               pcl::ModelCoefficients::Ptr &coefficients)
{
  pcl::SACSegmentation<pcl::PointNormal> segmentation;
  segmentation.setOptimizeCoefficients (true);
  segmentation.setModelType(pcl::SACMODEL_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setMaxIterations(500);
  segmentation.setDistanceThreshold(2);
  segmentation.setInputCloud(cloudnormals);
  segmentation.segment(*inliers, *coefficients);
}

void
extraction_plane ( pcl::PointCloud<pcl::PointNormal>::Ptr &cloudnormals,
                   pcl::PointIndices::Ptr &inliers,
                   pcl::PointCloud<pcl::PointNormal>::Ptr &points_out,
                   pcl::PointCloud<pcl::PointNormal>::Ptr &plane_out)
{

//  pcl::PointIndices::Ptr indices_out (new pcl::PointIndices);
  pcl::ExtractIndices<pcl::PointNormal> eifilter (false); // Initializing with true will allow us to extract the removed indices
  eifilter.setInputCloud (cloudnormals);
  eifilter.setIndices (inliers);
//  eifilter.getRemovedIndices (*indices_out);
  eifilter.setNegative (true);
  eifilter.filter(*points_out);
  eifilter.setNegative (false);
  eifilter.filter(*plane_out);
  std::cout<<"points_out : There are "<< points_out->points.size() << " data points" << endl;
  std::cout<<"plane_out : There are "<< plane_out->points.size() << " data points" << endl;
  pcl::visualization::PCLVisualizer viewer ("Cluster viewer");

//  viewer.addPointCloud(cloudnormals,pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> (cloudnormals, 255.0, 255.0, 255.0), "clustered points");
//  viewer.addPointCloud(plane_out,pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> (plane_out, 0.0, 255.0, 0.0), "clustered plane");
  viewer.addPointCloud(points_out,pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> (plane_out, 255.0, 0.0, 0.0), "cloud");
  viewer.spin ();
}
void
extraction_plane_( pcl::PointCloud<pcl::PointNormal>::Ptr &cloud,
                   pcl::PointIndices::Ptr &inliers,
                   pcl::ModelCoefficients::Ptr &coefficients,
                   pcl::PointCloud<pcl::PointNormal>::Ptr &points_out,
                   pcl::PointCloud<pcl::PointNormal>::Ptr &plane_out)
{
    float a,b,c,d;
    a = coefficients->values[0];
    b = coefficients->values[1];
    c = coefficients->values[2];
    d = coefficients->values[3];

    double  D;
    if (inliers->indices.size() > 0)
    {
      for (int i=0;i<cloud->size();i++)
      {			
        D=pcl::pointToPlaneDistance(cloud->points[i],a,b,c,d);			
        if ((D < 400)&&(D> 15))
        {
          points_out->push_back(cloud->points[i]);
        }
        else
        {
          plane_out->push_back(cloud->points[i]);
        }				
      }
    }
  pcl::visualization::PCLVisualizer viewer ("Cluster viewer");
//  viewer.addPointCloud(cloud,pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> (cloud, 255.0, 255.0, 255.0), "clustered points");
  viewer.addPointCloud(plane_out,pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> (plane_out, 255.0, 255.0, 0.0), "clustered plane");
  viewer.addPointCloud(points_out,pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> (points_out, 255.0, 255.0, 255.0), "cloud");
  viewer.spin ();
}


void
cluster_euclidean(pcl::PointCloud<pcl::PointNormal>::Ptr &cloud, 
                  std::vector<pcl::PointIndices> &cluster_indices)
{
	 
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>);
  tree->setInputCloud (cloud);
  pcl::EuclideanClusterExtraction<pcl::PointNormal> ec;
  ec.setClusterTolerance (4);
  ec.setMinClusterSize (1000);
  ec.setMaxClusterSize (50000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);
  std::cout << "Totally  " << cluster_indices.size() << " clusters " << std::endl;
}

bool identify_wall_in (pcl::PointCloud<pcl::PointNormal>::Ptr &points)
{
  int num_wall =0;
  int num_only_one_wall = 0;
  bool bool_wall_in = false;
  for (size_t j = 0; j < points->size(); j++)
  {
    const Eigen::Vector3f initial_normal (points->points[j].normal_x,points->points[j].normal_y,points->points[j].normal_z);
    const Eigen::Vector3f floor_normal (1.0,-1.0, 0.0);
    const Eigen::Vector3f wall_normal (0.0, 0.0, -1.0);
    float dot_product_f = fabsf (floor_normal.dot (initial_normal));
    float dot_product_w = fabsf (wall_normal.dot (initial_normal));
    if (dot_product_f < 0.5)                          num_wall ++ ;
    if (dot_product_w > 0.9)                          num_only_one_wall ++;
  }
  std::cout<<"[in] the number wall         : "<< num_wall <<"/"<<(points->size() /2.7) << "/"<< points->size() << std::endl;
  std::cout<<"[in] the number wall infront : "<< num_only_one_wall <<"/"<<(points->size() /2.5)<< "/"<< points->size() << std::endl;
  if (num_wall > 3000) //(points->size() /2.7))
  {
    if ( num_only_one_wall > (points->size() /2.5))
    {
      std::cout << " there is the only wall in front!! "<<std::endl;
      bool_wall_in =true;    
    }
  }
  return bool_wall_in;
}

bool identify_wall_le (pcl::PointCloud<pcl::PointNormal>::Ptr &points)
{
  int num_wall =0;
  int num_only_one_wall = 0;
  bool bool_wall_le = false;;
  float max_distance_x = max.x - min. x;
  Eigen::Vector4f centroid_pl;
  pcl::compute3DCentroid (*points, centroid_pl);
  for (size_t j = 0; j < points->size(); j++)
  {
    const Eigen::Vector3f initial_normal (points->points[j].normal_x,points->points[j].normal_y,points->points[j].normal_z);
    const Eigen::Vector3f floor_normal (1.0,-1.0, 0.0);
    const Eigen::Vector3f wall_normal (0.0, 0.0, -1.0);
    float dot_product_f = fabsf (floor_normal.dot (initial_normal));
    float dot_product_w = fabsf (wall_normal.dot (initial_normal));
    if (dot_product_f < 0.5)                          num_wall ++ ;
    if (dot_product_w > 0.9)                          num_only_one_wall ++;
  }
  std::cout<<"[le] the number wall         : "<< num_wall <<"/"<<(points->size() /2.7)<< "/"<< points->size() << std::endl;
  std::cout<<"[le] the number wall infront : "<< num_only_one_wall <<"/"<<(points->size() /2.2)<< "/"<< points->size() << std::endl;
  if (num_wall > 3000) // (points->size() /2.7))
  {
    if (centroid_pl[0] < max_distance_x /2 && num_only_one_wall < (points->size() /2.2))
    {
      std::cout << " there is the left wall!! "<<std::endl;
      bool_wall_le =true; 
    }
  }
  return bool_wall_le;
}

bool identify_wall_ri (pcl::PointCloud<pcl::PointNormal>::Ptr &points)
{
  int num_wall =0;
  int num_only_one_wall = 0;
  bool bool_wall_ri = false;;
  float max_distance_x = max.x - min. x;
  Eigen::Vector4f centroid_pl;
  pcl::compute3DCentroid (*points, centroid_pl);
  for (size_t j = 0; j < points->size(); j++)
  {
    const Eigen::Vector3f initial_normal (points->points[j].normal_x,points->points[j].normal_y,points->points[j].normal_z);
    const Eigen::Vector3f floor_normal (1.0,-1.0, 0.0);
    const Eigen::Vector3f wall_normal (0.0, 0.0, -1.0);
    float dot_product_f = fabsf (floor_normal.dot (initial_normal));
    float dot_product_w = fabsf (wall_normal.dot (initial_normal));
    if (dot_product_f < 0.5)                          num_wall ++ ;
    if (dot_product_w > 0.9)                          num_only_one_wall ++;
  }
  std::cout<<"[ri] the number wall         : "<< num_wall <<"/"<<(points->size() /2.7) << "/"<< points->size()<< std::endl;
  std::cout<<"[ri] the number wall infront : "<< num_only_one_wall <<"/"<<(points->size() /2.2) << "/"<< points->size()<< std::endl;
  if (num_wall > 3000)// (points->size() /2.7))
  {
    if (centroid_pl[0] > max_distance_x /2 && num_only_one_wall < (points->size() /2.2))
    {
      std::cout << " there is the right wall!! "<<std::endl;
      bool_wall_ri =true; 
    }
  }
  return bool_wall_ri;
}

bool identify_floor (pcl::PointCloud<pcl::PointNormal>::Ptr &points)
{
  int num_horiz = 0;
  int num_ceiling = 0;
  int num_floor = 0;
  bool bool_floor = false;;
  for (size_t j = 0; j < points->size(); j++)
  {
    const Eigen::Vector3f initial_normal (points->points[j].normal_x,points->points[j].normal_y,points->points[j].normal_z);
    const Eigen::Vector3f floor_normal (1.0,-1.0, 0.0);
    float dot_product_f = fabsf (floor_normal.dot (initial_normal));
    if (dot_product_f > 0.9)                          num_horiz ++;
    if (fabsf(points->points[j].y - min.y) < 30)  num_ceiling ++;
    if (fabsf(max.y - points->points[j].y) < 30)  num_floor ++;

  }


  std::cout<<"[fl] the number horizen      : "<< num_horiz  <<"/"<<(points->size() /2)<< "/"<< points->size() << std::endl;
  std::cout<<"[fl] the number floor        : "<< num_floor <<"/"<<(points->size() /2.5)<< "/"<< points->size() << std::endl;
  std::cout<<"[fl] the number ceiling      : "<< num_ceiling <<"/"<<(points->size() /1.5)<< "/"<< points->size() << std::endl;
  if (num_horiz > int ( points->size() /2))
  {
    if(num_ceiling < int (points->size() /1.5) && (num_floor > int ( points->size() /2.5)))
    {
      std::cout << " there is the floor!! "<<std::endl;
      bool_floor = true;
    }
  }

  return bool_floor;
}

bool identify_ceiling (pcl::PointCloud<pcl::PointNormal>::Ptr &points)
{
  int num_horiz = 0;
  int num_ceiling = 0;
  int num_floor = 0;
  bool bool_ce = false;;
  for (size_t j = 0; j < points->size(); j++)
  {
    const Eigen::Vector3f initial_normal (points->points[j].normal_x,points->points[j].normal_y,points->points[j].normal_z);
    const Eigen::Vector3f floor_normal (1.0,-1.0, 0.0);
    float dot_product_f = fabsf (floor_normal.dot (initial_normal));
    if (dot_product_f > 0.9)                      num_horiz ++;
    if (fabsf(points->points[j].y - min.y) < 30)  num_ceiling ++;
    if (fabsf(max.y - points->points[j].y) < 30)  num_floor ++;
  }

  std::cout<<"[ce] the number horizen      : "<< num_horiz <<"/"<<(points->size() /2) << "/"<< points->size() << std::endl;
  std::cout<<"[ce] the number floor        : "<< num_floor <<"/"<<(points->size() /1.5)<< "/"<< points->size() << std::endl;
  std::cout<<"[ce] the number ceiling      : "<< num_ceiling <<"/"<<(points->size() /2.5) << "/"<< points->size() << std::endl;
  if (num_horiz > int ( points->size() /2)) 
  {
    if(num_ceiling > int (points->size() /2.5) && (num_floor < int ( points->size() /1.5) ))
    {
      std::cout << " there is the ceiling!! "<<std::endl;
      bool_ce = true;
    }
  }
  return bool_ce;
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
  // Compute the features
  vfh_est.compute (*descriptors_out);
/*  pcl::visualization::PCLPlotter plotter;
  // We need to set the size of the descriptor beforehand.
  plotter.addFeatureHistogram(*descriptors_out, 308); 
  plotter.plot();
*/
}

void
find_feature_VFH_correspondences (pcl::PointCloud<pcl::VFHSignature308>::Ptr &source_descriptors,
                                  pcl::PointCloud<pcl::VFHSignature308>::Ptr &target_descriptors,
                                  float &distance)
{
  std::vector<float> correspondence_scores_out;
  correspondence_scores_out.resize (source_descriptors->size ());

  // Use a KdTree to search for the nearest matches in feature space
  pcl::search::KdTree<pcl::VFHSignature308> descriptor_kdtree;
  descriptor_kdtree.setInputCloud (target_descriptors);
  const int k = 1;
  std::vector<int> k_indices (k);
  std::vector<float> k_squared_distances (k);
  for (size_t i = 0; i < source_descriptors->size (); ++i)
  {
    descriptor_kdtree.nearestKSearch (*source_descriptors, i, k, k_indices, k_squared_distances);
    correspondence_scores_out[i] = k_squared_distances[0];
  }
    distance = correspondence_scores_out[0];
}

void
write_features (pcl::PointCloud<pcl::VFHSignature308>::Ptr &descriptors_vfh)
{
    ofstream outFile( "target_vfh_features.txt" );
    if ( !outFile ) 
    {
      printf("Error opening output file: %s!\n", "target_vfh_features.txt");
      exit( 1 );
    }
    for (size_t j = 0; j < descriptors_vfh->size (); j++)
    {
      outFile << "160 ";
      for(int q=0; q< 308; q++)
      {
        outFile << (q+1) << ":"<<descriptors_vfh->at(j).histogram[q] << " ";
      }
    } 
    outFile << std::endl;
    outFile.close();
}

void
write2svmdata(pcl::PointCloud<pcl::VFHSignature308>::Ptr &descriptors_vfh,
              const int count, std::vector<pcl::SVMData> &trainning_set)
{
  std::vector<pcl::SVMDataPoint> svm_set;
  for (size_t j = 0; j < descriptors_vfh->size (); j++)
  {
    for(int q=0; q< 308; q++)
    {
      pcl::SVMDataPoint svm_point;
      svm_point.idx = int (q+1);
      svm_point.value = descriptors_vfh->at(j).histogram[q];
      svm_set.push_back(svm_point);
    }
  }
  pcl::SVMData svm_data;
  svm_data.label = count;
  svm_data.SV = svm_set;
  trainning_set.push_back(svm_data);
}


void
detach_fields(pcl::PointCloud<pcl::PointNormal>::Ptr &points_,
              pcl::PointCloud<pcl::PointXYZ>::Ptr &mls_cloud_,
              pcl::PointCloud <pcl::Normal>::Ptr &mls_normals_)
{
  for (size_t i = 0; i < points_->size(); ++i) 
  {
    const pcl::PointNormal &mls_pt = points_->points[i];
    pcl::PointXYZ pt(mls_pt.x, mls_pt.y, mls_pt.z);
    pcl::Normal pt_(mls_pt.normal_x, mls_pt.normal_y, mls_pt.normal_z);
    mls_cloud_->push_back(pt);
    mls_normals_->push_back(pt_);
  }
}


pcl::PointCloud<pcl::PointXYZ>::Ptr runICP(pcl::PointCloud<pcl::PointXYZ>::Ptr eoCloud, 
                                           pcl::PointCloud<pcl::PointXYZ>::Ptr irCloud, 
                                           Eigen::Matrix4f& transformation) 
{
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  // Set the input source and target
  icp.setInputSource(irCloud);
  icp.setInputTarget(eoCloud);

  // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
//  icp.setMaxCorrespondenceDistance (200);
  // Set the maximum number of iterations (criterion 1)
  icp.setMaximumIterations (30);


  // Perform the alignment
  pcl::PointCloud<pcl::PointXYZ>::Ptr registered(new pcl::PointCloud<pcl::PointXYZ>());
  icp.align (*registered);
  cout<<"ICP has converged:"<<icp.hasConverged()<<" score: "<<icp.getFitnessScore()<<endl;
      
  // Obtain the transformation that aligned cloud_source to cloud_source_registered
  //Eigen::Matrix4f transformation = icp.getFinalTransformation ();
  transformation = icp.getFinalTransformation ();

  // Print results
  printf ("\n");
  pcl::console::print_info ("    | %6.3f %6.3f %6.3f %6.3f| \n", transformation (0,0), transformation (0,1), transformation (0,2), transformation (0,3));
  pcl::console::print_info ("R = | %6.3f %6.3f %6.3f %6.3f| \n", transformation (1,0), transformation (1,1), transformation (1,2), transformation (1,3));
  pcl::console::print_info ("    | %6.3f %6.3f %6.3f %6.3f| \n", transformation (2,0), transformation (2,1), transformation (2,2), transformation (2,3));
  pcl::console::print_info ("    | %6.3f %6.3f %6.3f %6.3f| \n", transformation (3,0), transformation (3,1), transformation (3,2), transformation (3,3));
  pcl::console::print_info ("\n");
  pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
  pcl::console::print_info ("\n");

  //report transform and error
  return registered;
}


int
main (int argc, char** argv)
{

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr cloudnormals (new pcl::PointCloud<pcl::PointNormal>);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  pcl::PointCloud<pcl::PointNormal>::Ptr plane_out (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr points_out (new pcl::PointCloud<pcl::PointNormal>);    
  
  pcl::PointIndices::Ptr inliers_(new pcl::PointIndices());
  pcl::ModelCoefficients::Ptr coefficients_(new pcl::ModelCoefficients());      
  pcl::PointCloud<pcl::PointNormal>::Ptr plane_out_ (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr points_out_ (new pcl::PointCloud<pcl::PointNormal>);

  pcl::PointIndices::Ptr inliers_s(new pcl::PointIndices());
  pcl::ModelCoefficients::Ptr coefficients_s(new pcl::ModelCoefficients());      
  pcl::PointCloud<pcl::PointNormal>::Ptr plane_out_s (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr points_out_s (new pcl::PointCloud<pcl::PointNormal>);

  pcl::PointCloud<pcl::PointNormal>::Ptr plane_wall_in (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr plane_wall_le (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr plane_wall_ri (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr plane_floor (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr plane_ceiling (new pcl::PointCloud<pcl::PointNormal>);
 

 if ( pcl::io::loadPCDFile <pcl::PointXYZ> (argv[1], *cloud) == -1)
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }

  //compute the maw and min value of this point cloud
  pcl::getMinMax3D (*cloud, min, max);
  
  // downsample the input point cloud
  const float voxel_grid_leaf_size = 4.0;
  downsample(cloud, voxel_grid_leaf_size,cloud_downsampled);
  std::cerr<<"number of cloud_downsampled : "<< cloud_downsampled->points.size() << " data points" << endl;

  // Compute the normals and create the points with normals
  const int neighbour_point = 50;
  compute_surface_normals(cloud_downsampled, neighbour_point, normals);
  pcl::concatenateFields(*cloud_downsampled, *normals, *cloudnormals);

  // Detect and extract the plane
  ransac_plane (cloudnormals, inliers, coefficients);
  extraction_plane_ (cloudnormals, inliers, coefficients, points_out, plane_out);

 // first detect the wall infront
  if ( identify_wall_in (plane_out) == true)
  {
    flag_wall_in =true;
    *plane_wall_in = *plane_out;

    // ** Second round
    ransac_plane (points_out, inliers_, coefficients_);
    extraction_plane_ (points_out, inliers_, coefficients_, points_out_, plane_out_);
    if ( identify_floor (plane_out_) == true )
    {
      flag_floor = true;
      *plane_floor = *plane_out_;
    }
    else if (identify_ceiling (plane_out_) == true )
    {
      flag_ceil = true;
      *plane_ceiling = *plane_out_;
    }
    else
    {
      *points_out_ = *points_out;
      *plane_out_  = *plane_out;
      std::cerr<<" Second round cannot detect the wall/floor/celing. "<<std::endl;
    }

    // ***** Third round
    ransac_plane (points_out_, inliers_s, coefficients_s);
    extraction_plane_ (points_out_, inliers_s, coefficients_s, points_out_s, plane_out_s);

    if ( identify_floor (plane_out_s) == true )
    {
      flag_floor = true;
      *plane_floor = *plane_out_s;
    }
    else if (identify_ceiling (plane_out_s) == true )
    {
      flag_ceil = true;
      *plane_ceiling = *plane_out_s;
    }
    else
    {
      *points_out_s = *points_out_;
      *plane_out_s  = *plane_out_;
      std::cerr<<" Third round cannot detect the wall/floor/celing. "<<std::endl;
    }
  }

  // first detect the floor
  else if ( identify_floor (plane_out) == true)
  {
    flag_floor = true;
    *plane_floor = *plane_out;

    // ** Second round    
    ransac_plane (points_out, inliers_, coefficients_);
    extraction_plane_ (points_out, inliers_, coefficients_, points_out_, plane_out_);
    if ( identify_wall_in (plane_out_) == true && plane_out_->size() > 4500)
    {
      flag_wall_in = true;
      *plane_wall_in = *plane_out_;
    }
    else if (identify_wall_le (plane_out_) == true && plane_out_->size() > 4500)
    {
      flag_wall_le = true;
      *plane_wall_le = *plane_out_;
    }
    else if (identify_wall_ri (plane_out_) == true && plane_out_->size() > 4500)
    {
      flag_wall_ri = true;
      *plane_wall_ri = *plane_out_;
    }
    else if (identify_ceiling (plane_out_) == true )
    {
      flag_ceil = true;
      *plane_ceiling = *plane_out_;
    }
    else
    {
      *points_out_ = *points_out;
      *plane_out_  = *plane_out;
      std::cerr<<" Second round cannot detect the wall/floor/celing. "<<std::endl;
    }

    // ***** Third round
    ransac_plane (points_out_, inliers_s, coefficients_s);
    extraction_plane_ (points_out_, inliers_s, coefficients_s, points_out_s, plane_out_s);

    if ( identify_wall_in (plane_out_s) == true && plane_out_s->size() > 4500  && flag_wall_le == false && flag_wall_ri == false)
    {
      flag_wall_in = true;
      *plane_wall_in = *plane_out_s;
    }
    else if (identify_wall_le (plane_out_s) == true && plane_out_s->size() > 4500)
    {
      flag_wall_le = true;
      *plane_wall_le = *plane_out_s;
    }
    else if (identify_wall_ri (plane_out_s) == true && plane_out_s->size() > 4500)
    {
      flag_wall_ri = true;
      *plane_wall_ri = *plane_out_s;
    }
    else if (identify_ceiling (plane_out_s) == true )
    {
      flag_ceil = true;
      *plane_ceiling = *plane_out_s;
    }
    else
    {
      *points_out_s = *points_out_;
      *plane_out_s  = *plane_out_;
      std::cerr<<" Third round cannot detect the wall/floor/celing. "<<std::endl;
    }

  }

  // first detect the left wall
  else if ( identify_wall_le (plane_out) == true)
  {
    flag_wall_le = true;
    *plane_wall_le = *plane_out;

    // ** Second round 
    ransac_plane (points_out, inliers_, coefficients_);
    extraction_plane_ (points_out, inliers_, coefficients_, points_out_, plane_out_);
    if ( identify_floor (plane_out_) == true )
    {
      flag_floor = true;
      *plane_floor = *plane_out_;
    }
    else if (identify_ceiling (plane_out_) == true )
    {
      flag_ceil = true;
      *plane_ceiling = *plane_out_;
    }
    else if (identify_wall_ri (plane_out_) == true && plane_out_->size() > 4500 )
    {
      flag_wall_ri = true;
      *plane_wall_ri = *plane_out_;
    }
    else
    {
      *points_out_ = *points_out;
      *plane_out_  = *plane_out;
    }

    // ***** Third round
    ransac_plane (points_out_, inliers_s, coefficients_s);
    extraction_plane_ (points_out_, inliers_s, coefficients_s, points_out_s, plane_out_s);

    if (flag_wall_ri = true  && identify_floor (plane_out_s) == true && plane_out_s->size() > 4500)
    {
      flag_wall_ri = true;
      *plane_wall_ri = *plane_out_s;
    }
    else if ( identify_floor (plane_out_s) == true )
    {
      flag_floor = true;
      *plane_floor = *plane_out_s;
    }
    else if (identify_ceiling (plane_out_s) == true )
    {
      flag_ceil = true;
      *plane_ceiling = *plane_out_s;
    }
    else
    {
      *points_out_s = *points_out_;
      *plane_out_s  = *plane_out_;
      std::cerr<<" Third round cannot detect the wall/floor/celing. "<<std::endl;
    }
  }

  // first detect the right wall
  else if ( identify_wall_ri (plane_out) == true)
  {
    flag_wall_ri = true;
    *plane_wall_ri = *plane_out;

    // ** Second round  
    ransac_plane (points_out, inliers_, coefficients_);
    extraction_plane_ (points_out, inliers_, coefficients_, points_out_, plane_out_);
    if ( identify_floor (plane_out_) == true )
    {
      flag_floor = true;
      *plane_floor = *plane_out_;
    }
    else if (identify_ceiling (plane_out_) == true )
    {
      flag_ceil = true;
      *plane_ceiling = *plane_out_;
    }
    else if (identify_wall_le (plane_out_) == true && plane_out_->size() > 4500 )
    {
      flag_wall_le = true;
      *plane_wall_le = *plane_out_;
    }
    else
    {
      *points_out_ = *points_out;
      *plane_out_  = *plane_out;
    }

    // ***** Third round
    ransac_plane (points_out_, inliers_s, coefficients_s);
    extraction_plane_ (points_out_, inliers_s, coefficients_s, points_out_s, plane_out_s);

    if (flag_wall_le = true && identify_floor (plane_out_s) == true && plane_out_s->size() > 4500)
    {
      flag_wall_le = true;
      *plane_wall_le = *plane_out_s;
    }
    else if ( identify_floor (plane_out_s) == true )
    {
      flag_floor = true;
      *plane_floor = *plane_out_s;
    }
    else if (identify_ceiling (plane_out_s) == true )
    {
      flag_ceil = true;
      *plane_ceiling = *plane_out_s;
    }
    else
    {
      *points_out_s = *points_out_;
      *plane_out_s  = *plane_out_;
      std::cerr<<" Third round cannot detect the wall/floor/celing. "<<std::endl;
    }
  }

  // first detect the ceiling
  else if ( identify_ceiling (plane_out) == true)
  {
    flag_ceil = true;
    *plane_ceiling = *plane_out;

    // ** Second round  
    ransac_plane (points_out, inliers_, coefficients_);
    extraction_plane_ (points_out, inliers_, coefficients_, points_out_, plane_out_);
    if ( identify_wall_in (plane_out_) == true )
    {
      flag_wall_in = true;
      *plane_wall_in = *plane_out_;
    }
    else if (identify_wall_le (plane_out_) == true && plane_out_->size() > 4500 )
    {
      flag_wall_le = true;
      *plane_wall_le = *plane_out_;
    }
    else if (identify_wall_ri (plane_out_) == true && plane_out_->size() > 4500 )
    {
      flag_wall_ri = true;
      *plane_wall_ri = *plane_out_;
    }
    else if ( identify_floor (plane_out_) == true )
    {
      flag_floor = true;
      *plane_floor = *plane_out_;
    }
    else
    {
      *points_out_ = *points_out;
      *plane_out_  = *plane_out;
    }

    // ***** Third round
    ransac_plane (points_out_, inliers_s, coefficients_s);
    extraction_plane_ (points_out_, inliers_s, coefficients_s, points_out_s, plane_out_s);
    if ( identify_wall_in (plane_out_s) == true && plane_out_s->size() > 4500 && flag_wall_le == false && flag_wall_ri == false)
    {
      flag_floor = true;
      *plane_floor = *plane_out_s;
    }
    else if (identify_wall_le (plane_out_s) == true && plane_out_s->size() > 4500 )
    {
      flag_wall_le = true;
      *plane_wall_le = *plane_out_s;
    }
    else if (identify_wall_ri (plane_out_s) == true && plane_out_s->size() > 4500 )
    {
      flag_wall_ri = true;
      *plane_wall_ri = *plane_out_s;
    }
    else if ( identify_floor (plane_out_s) == true )
    {
      flag_floor = true;
      *plane_floor = *plane_out_s;
    }
    else
    {
      *points_out_s = *points_out_;
      *plane_out_s  = *plane_out_;
      std::cerr<<" Third round cannot detect the wall/floor/celing. "<<std::endl;
    }
  }
  else
  {
    *points_out_s = *points_out;
    *plane_out_s  = *plane_out;
    std::cerr<<" First round cannot detect the wall/floor/celing, finish"<<std::endl;
    return (-1);
  }

  // load maquette vfh feature
  pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptors_model_vfh (new pcl::PointCloud<pcl::VFHSignature308>);
  pcl::io::loadPCDFile ("training_VFH_features_82.pcd", *descriptors_model_vfh);

/*  pcl::visualization::PCLPlotter plotter_;
  // We need to set the size of the descriptor beforehand.
  plotter_.addFeatureHistogram(*descriptors_model_vfh, 308); 
  plotter_.plot();
*/

  // cluster the point cloud
  std::vector<pcl::PointIndices> cluster_indices;
  cluster_euclidean(points_out_, cluster_indices);
  int count_ =0;
  int numero = 0;
  float distance_min =10000;
  pcl::PointCloud<pcl::PointNormal>::Ptr object_final (new pcl::PointCloud<pcl::PointNormal>);
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointNormal>::Ptr points_clustered (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptors_vfh (new pcl::PointCloud<pcl::VFHSignature308>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) 
    		points_clustered->points.push_back (points_out_->points[*pit]);

    pcl::PointCloud <pcl::Normal>::Ptr mls_normals (new pcl::PointCloud <pcl::Normal>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr mls_cloud (new pcl::PointCloud<pcl::PointXYZ>); 

    // Separate points XYZ and their normals
    detach_fields (points_clustered, mls_cloud, mls_normals);
    // compute the VFH and compare with the model
    compute_VFH_features (mls_cloud, mls_normals, descriptors_vfh);
    std::vector<float> correspondence_scores;
    float distance_;
    find_feature_VFH_correspondences(descriptors_vfh, descriptors_model_vfh, distance_);
    std::cout<<"the clustered cloud "<<count_<<"'s distances :"<<distance_<<std::endl;

    if (distance_ < distance_min )
    {
      distance_min = distance_;
      numero = count_;
      *object_final = *points_clustered;
    } 
    count_ ++; 
  }
   
  pcl::PointNormal min_obj;
  pcl::PointNormal max_obj;
  pcl::getMinMax3D (*object_final, min_obj, max_obj);

  // save the features to the file
  std::cout<<"the correct object is the index : " << numero <<std::endl;
  pcl::PointCloud <pcl::Normal>::Ptr obj_normals (new pcl::PointCloud <pcl::Normal>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr obj_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptors_obj (new pcl::PointCloud<pcl::VFHSignature308>);  
  detach_fields (object_final, obj_cloud, obj_normals);
  compute_VFH_features (obj_cloud, obj_normals, descriptors_obj);
  write_features(descriptors_obj);

  //svm classification
  // load the features dataset and object's features
  std::vector< std::vector<double> > out_;
  pcl::SVMClassify MyClassify_;

  MyClassify_.loadClassifierModel(argv[2]);
  MyClassify_.loadClassProblem("target_vfh_features.txt");
//  MyClassify.setInputTrainingSet (svmdata_obj);
  MyClassify_.classificationTest ();
  MyClassify_.getClassificationResult (out_);
  std::cout<< out_[0][0]<<std::endl;

  // load the predicted model from dataset
  pcl::PointCloud<pcl::PointXYZ>::Ptr points_maquette (new pcl::PointCloud<pcl::PointXYZ>);
  std::stringstream ss;
  int no_ = (int)out_[0][0];
  ss << "training_model_"<< no_ <<".pcd";
  pcl::io::loadPCDFile (ss.str(), *points_maquette);

  pcl::PointCloud<pcl::PointXYZ>::Ptr finalpoints (new pcl::PointCloud<pcl::PointXYZ>);
  Eigen::Matrix4f transform;
  finalpoints = runICP(obj_cloud, points_maquette, transform);

  std::cout<<"bool flag_floor : " << flag_floor<< std::endl;
  std::cout<<"bool flag_ceil : " << flag_ceil<< std::endl;
  std::cout<<"bool flag_wall : " << flag_wall<< std::endl;
  std::cout<<"bool flag_wall_in : " << flag_wall_in<< std::endl;
  std::cout<<"bool flag_wall_le : " << flag_wall_le<< std::endl;
  std::cout<<"bool flag_wall_ri : " << flag_wall_ri<< std::endl;

  // calculate the distance bewteen the wall infront and the object
  if (flag_wall_in)
  {
    Eigen::Vector4f centroid_obj;
    pcl::compute3DCentroid (*object_final, centroid_obj);
    Eigen::Vector4f centroid_wall;
    pcl::compute3DCentroid (*plane_wall_in, centroid_wall);
    pcl::PointXYZ p_object;
    p_object.x = centroid_obj[0];
    p_object.y = centroid_obj[1];
    p_object.z = centroid_obj[2];
    pcl::PointXYZ p_wall;
    p_wall.x = centroid_obj[0];
    p_wall.y = centroid_obj[1];
    p_wall.z = centroid_wall[2];
    float dis_wall_in = centroid_wall[2] - centroid_obj[2];
    std::cout<<"The distance between the wall and the object is "<<dis_wall_in<<std::endl;
    pcl::visualization::PCLVisualizer viz;
    viz.addPointCloud(cloudnormals, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> (cloudnormals, 255.0, 255.0, 255.0), "cloudnormals");
    viz.addPointCloud(plane_wall_in, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> (plane_wall_in, 255.0, 0.0, 0.0), "plane_wall");
//    viz.addPointCloud(finalpoints, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> (finalpoints, 0.0, 255.0, 255.0), "finalpoints");
    viz.addPointCloud(object_final, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> (object_final, 0.0, 0.0, 200.0), "object_final");
    viz.addCube (min_obj.x, max_obj.x, min_obj.y, max_obj.y, min_obj.z, max_obj.z, 0.0, 1.0, 1.0, "bounding box");
    viz.setRepresentationToWireframeForAllActors();
    viz.addPointCloud(plane_ceiling, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> (plane_ceiling, 0.0, 255.0, 0.0), "plane_ceiling");
    viz.addPointCloud(plane_floor, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> (plane_floor, 255.0, 255.0, 0.0), "floor");
    viz.addLine<pcl::PointXYZ, pcl::PointXYZ> (p_wall, p_object, 0.0, 255.0, 255.0, "line");
    viz.spin ();

  }
  else if (flag_wall_ri && flag_wall_le)
  {
    Eigen::Vector4f centroid_obj;
    pcl::compute3DCentroid (*object_final, centroid_obj);
    pcl::PointNormal min_le;
    pcl::PointNormal max_le;
    pcl::getMinMax3D (*plane_wall_le, min_le, max_le);
    pcl::PointNormal min_ri;
    pcl::PointNormal max_ri;
    pcl::getMinMax3D (*plane_wall_ri, min_ri, max_ri);
    float dis_corner ;
    pcl::PointXYZ p_object;
    pcl::PointXYZ p_corner;
    if ( (max_le.x - min_ri.x < 20 ) && (max_le.z - max_ri.z < 20) )
    {
      float corner_x = (max_le.x + min_ri.x)/2;
      float corner_z = (max_le.z + max_ri.z)/2;

      p_object.x = centroid_obj[0];
      p_object.y = centroid_obj[1];
      p_object.z = centroid_obj[2];

      p_corner.x = corner_x;
      p_corner.y = centroid_obj[1];
      p_corner.z = corner_z;
    }
    float squa_x = (p_corner.x - p_object.x)*(p_corner.x - p_object.x);
    float squa_z = (p_corner.z - p_object.z)*(p_corner.z - p_object.z);
    dis_corner = sqrtf(squa_x + squa_z);
    std::cout<<"The distance between the corner and the object is "<<dis_corner<<std::endl;
    pcl::visualization::PCLVisualizer viz;
    viz.addPointCloud(cloudnormals, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> (cloudnormals, 255.0, 255.0, 255.0), "cloudnormals");
    viz.addPointCloud(plane_wall_le, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> (plane_wall_le, 200.0, 0.0, 0.0), "plane_wall_left");
    viz.addPointCloud(object_final, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> (object_final, 0.0, 0.0, 200.0), "object_final");
    viz.addCube (min_obj.x, max_obj.x, min_obj.y, max_obj.y, min_obj.z, max_obj.z, 0.0, 1.0, 1.0, "bounding box");
    viz.setRepresentationToWireframeForAllActors();
    viz.addPointCloud(plane_wall_ri, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> (plane_wall_ri, 100.0, 0.0, 0.0), "plane_wall_right");
//    viz.addPointCloud(finalpoints, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> (finalpoints, 0.0, 255.0, 255.0), "finalpoints");
    viz.addPointCloud(plane_ceiling, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> (plane_ceiling, 0.0, 255.0, 0.0), "plane_ceiling");
    viz.addPointCloud(plane_floor, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> (plane_floor, 255.0, 255.0, 0.0), "floor");
    viz.addLine<pcl::PointXYZ, pcl::PointXYZ> (p_corner, p_object, 0.0, 255.0, 255.0, "line");
    viz.spin ();
  }
  // calculate the distance between the corner of two walls and the object
  else if (flag_wall_le)
  {
    Eigen::Vector4f centroid_obj;
    pcl::compute3DCentroid (*object_final, centroid_obj);
    Eigen::Vector4f centroid_wall;
    pcl::compute3DCentroid (*plane_wall_le, centroid_wall);
    pcl::PointXYZ p_object;
    p_object.x = centroid_obj[0];
    p_object.y = centroid_obj[1];
    p_object.z = centroid_obj[2];
    pcl::PointXYZ p_wall;
    p_wall.x = centroid_obj[0];
    p_wall.y = centroid_obj[1];
    p_wall.z = centroid_wall[2];
    float dis_wall_le = centroid_wall[2] - centroid_obj[2];
    std::cout<<"The distance between left wall and the object is "<<dis_wall_le<<std::endl;
    pcl::visualization::PCLVisualizer viz;
    viz.addPointCloud(cloudnormals, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> (cloudnormals, 255.0, 255.0, 255.0), "cloudnormals");
    viz.addPointCloud(plane_wall_le, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> (plane_wall_le, 255.0, 0.0, 0.0), "plane_wall");
    viz.addCube (min_obj.x, max_obj.x, min_obj.y, max_obj.y, min_obj.z, max_obj.z, 0.0, 1.0, 1.0, "bounding box");
    viz.setRepresentationToWireframeForAllActors();

    viz.addPointCloud(object_final, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> (object_final, 0.0, 0.0, 200.0), "object_final");
//    viz.addPointCloud(finalpoints, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> (finalpoints, 0.0, 255.0, 255.0), "finalpoints");
    viz.addPointCloud(plane_ceiling, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> (plane_ceiling, 0.0, 255.0, 0.0), "plane_ceiling");
    viz.addPointCloud(plane_floor, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> (plane_floor, 255.0, 255.0, 0.0), "floor");
    viz.addLine<pcl::PointXYZ, pcl::PointXYZ> (p_wall, p_object, 0.0, 255.0, 255.0, "line");
    viz.spin ();
  }
  else if (flag_wall_ri)
  {
    Eigen::Vector4f centroid_obj;
    pcl::compute3DCentroid (*object_final, centroid_obj);
    Eigen::Vector4f centroid_wall;
    pcl::compute3DCentroid (*plane_wall_ri, centroid_wall);
    pcl::PointXYZ p_object;
    p_object.x = centroid_obj[0];
    p_object.y = centroid_obj[1];
    p_object.z = centroid_obj[2];
    pcl::PointXYZ p_wall;
    p_wall.x = centroid_obj[0];
    p_wall.y = centroid_obj[1];
    p_wall.z = centroid_wall[2];
    float dis_wall_ri = centroid_wall[2] - centroid_obj[2];
    std::cout<<"The distance between right wall and the object is "<<dis_wall_ri<<std::endl; 
    pcl::visualization::PCLVisualizer viz;

    viz.addPointCloud(cloudnormals, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> (cloudnormals, 255.0, 255.0, 255.0), "cloudnormals");
    viz.addPointCloud(plane_wall_ri, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> (plane_wall_ri, 255.0, 0.0, 0.0), "plane_wall");
    viz.addPointCloud(object_final, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> (object_final, 0.0, 0.0, 200.0), "object_final");

    viz.addCube (min_obj.x, max_obj.x, min_obj.y, max_obj.y, min_obj.z, max_obj.z, 0.0, 1.0, 1.0, "bounding box");
    viz.setRepresentationToWireframeForAllActors();
    viz.addPointCloud(plane_ceiling, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> (plane_ceiling, 0.0, 255.0, 0.0), "plane_ceiling");
    viz.addPointCloud(plane_floor, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> (plane_floor, 255.0, 255.0, 0.0), "floor");
//    viz.addPointCloud(finalpoints, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> (finalpoints, 0.0, 255.0, 255.0), "finalpoints");
    viz.addLine<pcl::PointXYZ, pcl::PointXYZ> (p_wall, p_object, 0.0, 255.0, 255.0, "line");
    viz.spin ();

  }
  else std::cerr<<"Cannot calculate the distance between wall and object."<<std::endl; 

  return (0);
}

