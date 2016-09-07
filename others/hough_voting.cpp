#include <vector>
#include <iostream>
#include <Eigen/Core>
#include <pcl/common/time.h>
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include <pcl/correspondence.h>
#include "pcl/io/pcd_io.h"
#include <pcl/search/kdtree.h>
#include "pcl/kdtree/kdtree_flann.h"
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/transformation_estimation_svd_scale.h>
#include <pcl/features/shot_omp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/icp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/console/parse.h>
#include <pcl/recognition/hv/hv_go.h>


float cg_size_ (15.0f);
float cg_thresh_ (2.0f);
int icp_max_iter_ (5);
float icp_corr_distance_ (0.01f);
float hv_clutter_reg_ (5.0f);
float hv_inlier_th_ (0.01f);
float hv_occlusion_th_ (0.02f);
float hv_rad_clutter_ (0.05f);
float hv_regularizer_ (3.0f);
float hv_rad_normals_ (0.08);
bool hv_detect_clutter_ (true);


std::string model_filename_;
std::string scene_filename_;


void
showHelp (char *filename)
{
  std::cout << std::endl;
  std::cout << "***************************************************************************" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "*             Correspondence Grouping Tutorial - Usage Guide              *" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "***************************************************************************" << std::endl << std::endl;
  std::cout << "Usage: " << filename << " model_filename.pcd scene_filename.pcd [Options]" << std::endl << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "     --cg_size val:          Cluster size (default "<< cg_size_ << ")"  << std::endl;
  std::cout << "     --cg_thresh val:        Clustering threshold (default " << cg_thresh_ << ")" << std::endl;
  std::cout << "     --icp_max_iter val:          ICP max iterations number (default " << icp_max_iter_ << ")" << std::endl;
  std::cout << "     --icp_corr_distance val:     ICP correspondence distance (default " << icp_corr_distance_ << ")" << std::endl << std::endl;
  std::cout << "     --hv_clutter_reg val:        Clutter Regularizer (default " << hv_clutter_reg_ << ")" << std::endl;
  std::cout << "     --hv_inlier_th val:          Inlier threshold (default " << hv_inlier_th_ << ")" << std::endl;
  std::cout << "     --hv_occlusion_th val:       Occlusion threshold (default " << hv_occlusion_th_ << ")" << std::endl;
  std::cout << "     --hv_rad_clutter val:        Clutter radius (default " << hv_rad_clutter_ << ")" << std::endl;
  std::cout << "     --hv_regularizer val:        Regularizer value (default " << hv_regularizer_ << ")" << std::endl;
  std::cout << "     --hv_rad_normals val:        Normals radius (default " << hv_rad_normals_ << ")" << std::endl;
  std::cout << "     --hv_detect_clutter val:     TRUE if clutter detect enabled (default " << hv_detect_clutter_ << ")" << std::endl << std::endl;
}


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


void
compute_SHOT_features (pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr &points,
                       pcl::PointCloud<pcl::Normal>::Ptr &normals,
                       float feature_radius,
                       pcl::PointCloud<pcl::SHOT352>::Ptr &descriptors_out)
{

  // Create a PFHEstimation object
  pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::SHOT352, pcl::ReferenceFrame> shot_est;

  // Set it to use a FLANN-based KdTree to perform its neighborhood searches
  shot_est.setSearchMethod (pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));

  // Specify the radius of the PFH feature
  shot_est.setRadiusSearch (feature_radius);

  shot_est.setSearchSurface (cloud);
  
  // Set the input points and surface normals
  shot_est.setInputCloud (points);
  shot_est.setInputNormals (normals);
  std::cout << "start compute SHOT features descriptors" << std::endl;
  // Compute the features
  shot_est.compute (*descriptors_out);
  std::cout << "Compute SHOT descriptors successfully, the number of descriptors is " << descriptors_out->size() << std::endl;
}

void estimation_feature_correspondences_SHOT(pcl::PointCloud<pcl::SHOT352>::Ptr &source_descriptors,
                              pcl::PointCloud<pcl::SHOT352>::Ptr &target_descriptors, pcl::CorrespondencesPtr &correspondences_all)
{
  PCL_INFO("Finding correspondences...\n");
  pcl::registration::CorrespondenceEstimation<pcl::SHOT352, pcl::SHOT352> ce;
  ce.setInputSource (source_descriptors);
  ce.setInputTarget (target_descriptors);
  ce.determineReciprocalCorrespondences (*correspondences_all);
  PCL_INFO("%d correspondences found\n", correspondences_all->size ());
}

void
find_feature_SHOT_correspondences (pcl::PointCloud<pcl::SHOT352>::Ptr &model_descriptors,
                                   pcl::PointCloud<pcl::SHOT352>::Ptr &scene_descriptors,
                                   pcl::CorrespondencesPtr &model_scene_corrs)
{

  pcl::KdTreeFLANN<pcl::SHOT352> match_search;
  match_search.setInputCloud (model_descriptors);

  //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
  for (size_t i = 0; i < scene_descriptors->size (); ++i)
  {
    std::vector<int> neigh_indices (1);
    std::vector<float> neigh_sqr_dists (1);
    if (!pcl_isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs
    {
      continue;
    }
    int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
    if(found_neighs == 1 && neigh_sqr_dists[0] < 0.20f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
    {
      pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
      model_scene_corrs->push_back (corr);
    }
  }
  std::cout << "SHOT Correspondences found: " << model_scene_corrs->size () << std::endl;

}

void rejection_correspondences(pcl::PointCloud<pcl::PointXYZ>::Ptr &points1, pcl::PointCloud<pcl::PointXYZ>::Ptr &points2,
                               pcl::CorrespondencesPtr &correspondences_all, pcl::CorrespondencesPtr &correspondences_inliers)
{

  pcl::CorrespondencesPtr correspondences_x (new pcl::Correspondences);
  pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> sac;
  sac.setInputSource (points1);
  sac.setInputTarget (points2);
  sac.setInlierThreshold (0.1);
  sac.setMaximumIterations (1000);
  sac.setInputCorrespondences (correspondences_all);
  sac.getCorrespondences (*correspondences_inliers);

  PCL_INFO("%d inliers correspondences\n", correspondences_inliers->size ());
/*
  for (size_t i=0; i < correspondences_x -> size (); i++){
    if (correspondences_x ->at(i).distance < 50.0f) {
      correspondences_inliers->push_back(correspondences_x->at(i));
    }
  }

  PCL_INFO("%d inliers correspondences\n", correspondences_inliers->size ());
  for(size_t i =0; i< correspondences_inliers->size ();i++){
    std::cerr<<"the linliers correspondences"<< correspondences_inliers->at(i)<<std::endl;
  }
*/
}

void rf_calculation  (pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr &points,
                      pcl::PointCloud<pcl::Normal>::Ptr &normals,
                      pcl::PointCloud<pcl::ReferenceFrame>::Ptr &reference_frame)
{
  pcl::BOARDLocalReferenceFrameEstimation<pcl::PointXYZ, pcl::Normal, pcl::ReferenceFrame> rf_est;
  rf_est.setFindHoles (true);
  rf_est.setRadiusSearch (0.05f);

  rf_est.setInputCloud (points);
  rf_est.setInputNormals (normals);
  rf_est.setSearchSurface (cloud);
  rf_est.compute (*reference_frame);
} 
                      
void Hough_groupping  (float &cg_size,
                       float &cg_thresh,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr &model,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr &scene,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr &model_keypoints,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr &scene_keypoints,
                       pcl::PointCloud<pcl::ReferenceFrame>::Ptr &model_rf,
                       pcl::PointCloud<pcl::ReferenceFrame>::Ptr &scene_rf,                      
                       pcl::CorrespondencesPtr &model_scene_corrs,
                       pcl::CorrespondencesPtr & group_corrs,
                       Eigen::Matrix4f & transforms_hough)
{

  std::cout << "cg_size: " << cg_size << "  cg_thresh: " << cg_thresh << std::endl;
  
    //  Clustering
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transformations;
  std::vector<pcl::Correspondences> clustered_corrs;

  pcl::Hough3DGrouping<pcl::PointXYZ, pcl::PointXYZ, pcl::ReferenceFrame, pcl::ReferenceFrame> clusterer;
  clusterer.setHoughBinSize (cg_size);//25
  clusterer.setHoughThreshold (cg_thresh);//15
  clusterer.setUseInterpolation (true);
  clusterer.setUseDistanceWeight (false);

  clusterer.setInputCloud (model_keypoints);
  clusterer.setInputRf (model_rf);
  clusterer.setSceneCloud (scene_keypoints);
  clusterer.setSceneRf (scene_rf);
  clusterer.setModelSceneCorrespondences (model_scene_corrs);
  
    //clusterer.cluster (clustered_corrs);
  clusterer.recognize (transformations, clustered_corrs);
  std::cout << "Model instances found: " << transformations.size () << std::endl;

  int num_cor=0;
  int no_max=0;

  for (size_t i = 0; i < transformations.size(); i++)
  {  
    std::cout << "Instance " << (i + 1) << ":" << std::endl;
    std::cout << "\tHas " << clustered_corrs[i].size() << " correspondences." << std::endl << std::endl;
	
    if(num_cor <= clustered_corrs[i].size ()) 
    {
      num_cor = clustered_corrs[i].size ();
      no_max = i;
    }

									
    Eigen::Matrix3f rotation = transformations[i].block<3, 3>(0, 0);
    Eigen::Vector3f translation = transformations[i].block<3, 1>(0, 3);
    printf("\t\t    | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
    printf("\t\tR = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
    printf("\t\t    | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
    std::cout << std::endl;
    printf("\t\tt = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));

    std::cout << "\t We choice this one :  Index "<< no_max<<" has" << clustered_corrs[no_max].size() << " correspondences." << std::endl << std::endl;

    Eigen::Matrix4f transforms_hough_;
    pcl::CorrespondencesPtr group_corrs_ (new pcl::Correspondences);

    transforms_hough_ = transformations[i];//.block<4,4>(0, 0);
    for (size_t j = 0; j < clustered_corrs[i].size (); ++j)
      group_corrs_->push_back(clustered_corrs[i][j]);

    pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints_trans (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud  (*model_keypoints, *model_keypoints_trans, transforms_hough_);
    // Show grouping
    pcl::visualization::PCLVisualizer visu("every grouping");
    visu.addPointCloud (model_keypoints_trans, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> (model_keypoints_trans, 255.0, 0.0, 0.0), "model");
    visu.addPointCloud (scene_keypoints, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> (scene_keypoints, 255.0, 255.0, 255.0), "scene_keypoints");
    visu.spin ();

  }
  transforms_hough = transformations[no_max];//.block<4,4>(0, 0);
  for (size_t j = 0; j < clustered_corrs[no_max].size (); ++j)
    group_corrs->push_back(clustered_corrs[no_max][j]);

  std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> instances;
  for (size_t i = 0; i < transformations.size(); ++i)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_model (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud (*model, *rotated_model, transformations[i]);
    instances.push_back (rotated_model);
  }

  /**
   * ICP
   */
  std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> registered_instances;
  if (true)
  {
    cout << "--- ICP ---------" << endl;

    for (size_t i = 0; i < transformations.size (); ++i)
    {
      pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
      icp.setMaximumIterations (icp_max_iter_);
      icp.setMaxCorrespondenceDistance (icp_corr_distance_);
      icp.setInputTarget (scene);
      icp.setInputSource (instances[i]);
      pcl::PointCloud<pcl::PointXYZ>::Ptr registered (new pcl::PointCloud<pcl::PointXYZ>);
      icp.align (*registered);
      registered_instances.push_back (registered);
      cout << "Instance " << i << " ";
      if (icp.hasConverged ())
      {
        cout << "Aligned!" << endl;
      }
      else
      {
        cout << "Not Aligned!" << endl;
      }
    }

    cout << "-----------------" << endl << endl;
  }
/***

   // Hypothesis Verification
   
  cout << "--- Hypotheses Verification ---" << endl;
  std::vector<bool> hypotheses_mask;  // Mask Vector to identify positive hypotheses

  pcl::GlobalHypothesesVerification<pcl::PointXYZ, pcl::PointXYZ> GoHv;

  GoHv.setSceneCloud (scene);  // Scene Cloud
  GoHv.addModels (registered_instances, true);  //Models to verify
  GoHv.addCompleteModels(registered_instances); 

  GoHv.setInlierThreshold (hv_inlier_th_);
  GoHv.setOcclusionThreshold (hv_occlusion_th_);
  GoHv.setRegularizer (hv_regularizer_);
  GoHv.setRadiusClutter (hv_rad_clutter_);
  GoHv.setClutterRegularizer (hv_clutter_reg_);
  GoHv.setDetectClutter (hv_detect_clutter_);
  GoHv.setRadiusNormals (hv_rad_normals_);

  GoHv.verify ();
  GoHv.getMask (hypotheses_mask);  // i-element TRUE if hvModels[i] verifies hypotheses

  for (int i = 0; i < hypotheses_mask.size (); i++)
  {
    if (hypotheses_mask[i])
    {
      cout << "Instance " << i << " is GOOD! <---" << endl;
    }
    else
    {
      cout << "Instance " << i << " is bad!" << endl;
    }
  }
  cout << "-------------------------------" << endl;
*/

}


void geometric_consistency (float &cg_size,
                            float &cg_thresh,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr &model_keypoints,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr &scene_keypoints,                     
                            pcl::CorrespondencesPtr &model_scene_corrs,
                            pcl::CorrespondencesPtr & group_corrs,
                            Eigen::Matrix4f & transforms_geo)

{

  std::cout << "cg_size: " << cg_size << "cg_thresh: " << cg_thresh << std::endl;
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transformations;
  std::vector<pcl::Correspondences> clustered_corrs;

  pcl::GeometricConsistencyGrouping<pcl::PointXYZ, pcl::PointXYZ> gc_clusterer;
  gc_clusterer.setGCSize (cg_size);
  gc_clusterer.setGCThreshold (cg_thresh);

  gc_clusterer.setInputCloud (model_keypoints);
  gc_clusterer.setSceneCloud (scene_keypoints);
  gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

  //gc_clusterer.cluster (clustered_corrs);
  gc_clusterer.recognize (transformations, clustered_corrs);

  int num_cor=0;
  int no_max=0;

  std::cout << "Model instances found: " << transformations.size () << std::endl;
  for (size_t i = 0; i < transformations.size(); i++)
  {  

    if(num_cor <= clustered_corrs[i].size ()) {
      num_cor = clustered_corrs[i].size ();
      no_max = i;
    }

    std::cout << "Instance " << (i + 1) << ":" << std::endl;
    std::cout << "\tHas " << clustered_corrs[i].size() << " correspondences." << std::endl << std::endl;
										
    Eigen::Matrix3f rotation = transformations[i].block<3, 3>(0, 0);
    Eigen::Vector3f translation = transformations[i].block<3, 1>(0, 3);
    printf("\t\t    | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
    printf("\t\tR = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
    printf("\t\t    | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
    std::cout << std::endl;
    printf("\t\tt = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));
    std::cout << "\t We choice this one :  Index "<< no_max<<" has" << clustered_corrs[no_max].size() << " correspondences." << std::endl << std::endl;

    Eigen::Matrix4f transforms_geo_;
    pcl::CorrespondencesPtr group_corrs_ (new pcl::Correspondences);

    transforms_geo_ = transformations[i].block<4,4>(0, 0);
    for (size_t j = 0; j < clustered_corrs[i].size (); ++j)
      group_corrs_->push_back(clustered_corrs[i][j]);

    pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints_trans (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud  (*model_keypoints, *model_keypoints_trans, transforms_geo_);
    // Show grouping
    pcl::visualization::PCLVisualizer visu("every grouping");
    visu.addPointCloud (model_keypoints_trans, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> (model_keypoints_trans, 255.0, 0.0, 0.0), "model");
    visu.addPointCloud (scene_keypoints, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> (scene_keypoints, 255.0, 255.0, 255.0), "scene_keypoints");
    visu.spin ();

    transforms_geo = transformations[no_max].block<4,4>(0, 0);
    for (size_t j = 0; j < clustered_corrs[no_max].size (); ++j)
      group_corrs->push_back(clustered_corrs[no_max][j]);
  }
/*
  int num_cor=0;
  int no_max=0;


  for (size_t i = 0; i < transformations.size(); i++)
  {  
    std::cout << "Instance " << (i + 1) << ":" << std::endl;
    std::cout << "\tHas " << clustered_corrs[i].size() << " correspondences." << std::endl << std::endl;

    if(num_cor <= clustered_corrs[i].size ()) {
      num_cor = clustered_corrs[i].size ();
      no_max = i;
    }


    Eigen::Matrix3f rotation = transformations[i].block<3, 3>(0, 0);
    Eigen::Vector3f translation = transformations[i].block<3, 1>(0, 3);
    printf("\t\t    | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
    printf("\t\tR = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
    printf("\t\t    | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
    std::cout << std::endl;
    printf("\t\tt = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));
  }

  std::cout << "\t We choice this one :  Index "<< no_max<<" has" << clustered_corrs[no_max].size() << " correspondences." << std::endl << std::endl;


  transforms_geo = transformations[no_max].block<4,4>(0, 0);
  for (size_t j = 0; j < clustered_corrs[no_max].size (); ++j)
    group_corrs->push_back(clustered_corrs[no_max][j]);

*/

}


void translation_only (pcl::PointCloud<pcl::PointXYZ>::Ptr &points1,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr &points2,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud1,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr &points1_trans,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud1_trans) 
{

//  Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity(); 
  Eigen::Vector4f centroid1;
  pcl::compute3DCentroid (*points1, centroid1);
  Eigen::Vector4f centroid2;
  pcl::compute3DCentroid (*points2, centroid2);
  Eigen::Vector4f translation_ = centroid2 - centroid1;
  
  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

  // Define a translation of 2.5 meters on the x axis.
  transform_2.translation() << translation_[0], translation_[1],translation_[2];

  pcl::transformPointCloud (*points1, *points1_trans, transform_2);
  pcl::transformPointCloud (*cloud1, *cloud1_trans, transform_2);

}




void calculate_transformation (pcl::PointCloud<pcl::PointXYZ>::Ptr &points1,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr &points2,
                               pcl::CorrespondencesPtr &correspondences_inliers,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr &points1_trans)
{
/*
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

  for (size_t i = 0; i < correspondences_inliers->size (); ++i)
  {
     cloud_in->push_back(points1-> points[correspondences_inliers->at(i).index_query]);
     cloud_out->push_back(points1-> points[correspondences_inliers->at(i).index_query]);
  }
  
  pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ> trans_SVD;
  pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ>::Matrix4 transformation2;
  trans_SVD.estimateRigidTransformation (*cloud_in,*cloud_out,transformation2);
*/

  pcl::registration::TransformationEstimationSVDScale<pcl::PointXYZ,pcl::PointXYZ> trans_SVD;
  pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,pcl::PointXYZ>::Matrix4 transformation2;
  trans_SVD.estimateRigidTransformation (*points1,*points2, *correspondences_inliers, transformation2);

  std::cout << "The Estimated Rotation and translation matrices (using getTransformation function) are : \n" << std::endl;
  printf ("\n");
  printf ("    | %6.3f %6.3f %6.3f %6.3f| \n", transformation2 (0,0), transformation2 (0,1), transformation2 (0,2), transformation2 (0,3));
  printf ("R = | %6.3f %6.3f %6.3f %6.3f| \n", transformation2 (1,0), transformation2 (1,1), transformation2 (1,2), transformation2 (1,3));
  printf ("    | %6.3f %6.3f %6.3f %6.3f| \n", transformation2 (2,0), transformation2 (2,1), transformation2 (2,2), transformation2 (2,3));
  printf ("    | %6.3f %6.3f %6.3f %6.3f| \n", transformation2 (3,0), transformation2 (3,1), transformation2 (3,2), transformation2 (3,3));
  printf ("\n");
  printf ("t = < %0.3f, %0.3f, %0.3f >\n", transformation2 (0,3), transformation2 (1,3), transformation2 (2,3));
  PCL_INFO("start transform\n");
  pcl::transformPointCloud (*points1, *points1_trans, transformation2);
  
}

void alignement (pcl::PointCloud<pcl::PointXYZ>::Ptr &points1,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr &points2,
                 pcl::PointCloud<pcl::SHOT352>::Ptr &points1_descriptors,
                 pcl::PointCloud<pcl::SHOT352>::Ptr &points2_descriptors,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr &object_aligned )
{
  const float leaf = 1.6f;
  pcl::console::print_highlight ("Starting alignment...\n");
  pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, pcl::SHOT352> align;
  align.setInputSource (points1);
  align.setSourceFeatures (points1_descriptors);
  align.setInputTarget (points2);
  align.setTargetFeatures (points2_descriptors);
  align.setMaximumIterations (50000); // Number of RANSAC iterations
  align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness (5); // Number of nearest features to use
  align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
  align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
  {
    pcl::ScopeTime t("Alignment");
    align.align (*object_aligned);
  }
  
  if (align.hasConverged ())
  {
    // Print results
    printf ("\n");
    Eigen::Matrix4f transformation = align.getFinalTransformation ();
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
    pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), points1->size ());
    
    // Show alignment
    pcl::visualization::PCLVisualizer visu("Alignment");
    visu.addPointCloud (points2, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> (points2, 0.0, 255.0, 0.0), "model");
    visu.addPointCloud (object_aligned, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> (object_aligned, 0.0, 0.0, 255.0), "object_aligned");
    visu.spin ();
  }
  else
  {
    pcl::console::print_error ("Alignment failed!\n");
  }
  

}


pcl::PointCloud<pcl::PointXYZ>::Ptr runICP(pcl::PointCloud<pcl::PointXYZ>::Ptr irCloud,
                                           pcl::PointCloud<pcl::PointXYZ>::Ptr eoCloud, 
                                           Eigen::Matrix4f& transform) {

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
     
  // Set the input source and target
  icp.setInputSource(irCloud);
  icp.setInputTarget(eoCloud);

  // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
//  icp.setMaxCorrespondenceDistance (0.1);
  // Set the maximum number of iterations (criterion 1)
  icp.setMaximumIterations (5000);
  // Set the transformation epsilon (criterion 2)
//  icp.setTransformationEpsilon (1e-8);
  // Set the euclidean distance difference epsilon (criterion 3)
//  icp.setEuclideanFitnessEpsilon (1);

  // Perform the alignment
  pcl::PointCloud<pcl::PointXYZ>::Ptr registered(new pcl::PointCloud<pcl::PointXYZ>());
  icp.align (*registered);
  cout<<"ICP has converged:"<<icp.hasConverged()<<" score: "<<icp.getFitnessScore()<<endl;
      
  // Obtain the transformation that aligned cloud_source to cloud_source_registered
  //Eigen::Matrix4f transformation = icp.getFinalTransformation ();
  transform = icp.getFinalTransformation ();

  //report transform and error
  return registered;
}


void visualize_correspondences (const pcl::PointCloud<pcl::PointXYZ>::Ptr points1,
                                const pcl::PointCloud<pcl::PointXYZ>::Ptr points2,
                                const pcl::CorrespondencesPtr &correspondences_all)
{
  // We want to visualize two clouds side-by-side, so do to this, we'll make copies of the clouds and transform them
  // by shifting one to the left and the other to the right.  Then we'll draw lines between the corresponding points

  // Create some new point clouds to hold our transformed data
  pcl::PointCloud<pcl::PointXYZ>::Ptr points_left (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr points_right (new pcl::PointCloud<pcl::PointXYZ>);

  // Shift the first clouds' points to the left
  //const Eigen::Vector3f translate (0.0, 0.0, 0.3);
  const Eigen::Vector3f translate (0.5, 0.0, 0.0);
  const Eigen::Quaternionf no_rotation (0, 0, 0, 0);

  pcl::transformPointCloud (*points1, *points_left, -translate, no_rotation);

  // Shift the second clouds' points to the right
  pcl::transformPointCloud (*points2, *points_right, translate, no_rotation);

  // Add the clouds to the vizualizer
  pcl::visualization::PCLVisualizer viz;
  viz.addPointCloud (points_left, "points_left");
  viz.addPointCloud (points_right, "points_right");

  // Draw lines between the best corresponding points
  PCL_INFO("%d correspondences found\n", correspondences_all->size ());

  for (size_t i = 0; i < correspondences_all->size (); ++i)
  {

    // Get the pair of points
    const pcl::PointXYZ & p_left = points_left-> points[correspondences_all->at(i).index_query];
    const pcl::PointXYZ & p_right = points_right->points[correspondences_all->at(i).index_match];

    // Generate a random (bright) color
    double r = (rand() % 100);
    double g = (rand() % 100);
    double b = (rand() % 100);
    double max_channel = std::max (r, std::max (g, b));
    r /= max_channel;
    g /= max_channel;
    b /= max_channel;

    // Generate a unique string for each line
    std::stringstream ss ("line");
    ss << i;
    // Draw the line
//    if (i % 7 == 0){
      viz.addLine<pcl::PointXYZ, pcl::PointXYZ> (p_left, p_right, r, g, b, ss.str ());
//    }
  }

  // Give control over to the visualizer
  viz.spin ();
}
                               		 
void visualize_transformation(const pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
                              const pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud)
{

  printf(  "\nPoint cloud colors :  white  = original point cloud\n"
      "                        red  = transformed point cloud\n");

  pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");

   // Define R,G,B colors for the point cloud
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (source_cloud, 255, 255, 255);
  // We add the point cloud to the viewer and pass the color handler
  viewer.addPointCloud (source_cloud, source_cloud_color_handler, "original_cloud");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (transformed_cloud, 230, 20, 20); // Red
  viewer.addPointCloud (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

  viewer.addCoordinateSystem (1.0, "cloud", 0);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
  //viewer.setPosition(800, 400); // Setting visualiser window position

  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce ();
  }


}

void
parseCommandLine (int argc, char *argv[])
{
  //Show help
  if (pcl::console::find_switch (argc, argv, "-h"))
  {
    showHelp (argv[0]);
    exit (0);
  }

  //Model & scene filenames
  std::vector<int> filenames;
  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  if (filenames.size () != 2)
  {
    std::cout << "Filenames missing.\n";
    showHelp (argv[0]);
    exit (-1);
  }

  model_filename_ = argv[filenames[0]];
  scene_filename_ = argv[filenames[1]];

  //General parameters
  pcl::console::parse_argument (argc, argv, "--cg_size", cg_size_);
  pcl::console::parse_argument (argc, argv, "--cg_thresh", cg_thresh_);
  pcl::console::parse_argument (argc, argv, "--icp_max_iter", icp_max_iter_);
  pcl::console::parse_argument (argc, argv, "--icp_corr_distance", icp_corr_distance_);
  pcl::console::parse_argument (argc, argv, "--hv_clutter_reg", hv_clutter_reg_);
  pcl::console::parse_argument (argc, argv, "--hv_inlier_th", hv_inlier_th_);
  pcl::console::parse_argument (argc, argv, "--hv_occlusion_th", hv_occlusion_th_);
  pcl::console::parse_argument (argc, argv, "--hv_rad_clutter", hv_rad_clutter_);
  pcl::console::parse_argument (argc, argv, "--hv_regularizer", hv_regularizer_);
  pcl::console::parse_argument (argc, argv, "--hv_rad_normals", hv_rad_normals_);
  pcl::console::parse_argument (argc, argv, "--hv_detect_clutter", hv_detect_clutter_);
}


int main (int argc, char **argv[])
{

  parseCommandLine (argc, argv);
  static double first = pcl::getTime();
  // Create some new point clouds to hold our data
  pcl::PointCloud<pcl::PointXYZ>::Ptr points1 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled1 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr normals_all1 (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Normal>::Ptr normals1 (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::SHOT352>::Ptr descriptors_shot1 (new pcl::PointCloud<pcl::SHOT352>);

  pcl::PointCloud<pcl::PointXYZ>::Ptr points2 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled2 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr normals_all2 (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Normal>::Ptr normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::SHOT352>::Ptr descriptors_shot2 (new pcl::PointCloud<pcl::SHOT352>);

  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled1_trans (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr object_aligned (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled2_trans (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr normals1_trans (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled2_move (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr points2_move (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::CorrespondencesPtr correspondences_all (new pcl::Correspondences);
  pcl::CorrespondencesPtr correspondences_inliers (new pcl::Correspondences);

  pcl::CorrespondencesPtr correspondences_hough (new pcl::Correspondences);
  pcl::CorrespondencesPtr correspondences_inliers_ (new pcl::Correspondences);

  pcl::PointCloud<pcl::ReferenceFrame>::Ptr model_rf (new pcl::PointCloud<pcl::ReferenceFrame> ());
  pcl::PointCloud<pcl::ReferenceFrame>::Ptr scene_rf (new pcl::PointCloud<pcl::ReferenceFrame> ());
  Eigen::Matrix4f group_transform;


  //  Load clouds
  if (pcl::io::loadPCDFile (model_filename_, *points1) < 0)
  {
    std::cout << "Error loading model cloud." << std::endl;
    showHelp (argv[0]);
    return (-1);
  }
  if (pcl::io::loadPCDFile (scene_filename_, *points2) < 0)
  {
    std::cout << "Error loading scene cloud." << std::endl;
    showHelp (argv[0]);
    return (-1);
  }

  std::cout << "load  " << model_filename_
            << "and " << scene_filename_ << "successfully" << std::endl;

  // Downsample the cloud
  const float voxel_grid_leaf_size1 = 6.0;
  const float voxel_grid_leaf_size2 = 4.0;
  downsample (points1, voxel_grid_leaf_size1, downsampled1);
  downsample (points2, voxel_grid_leaf_size2, downsampled2);

  std::cout << "downsample points1 :" << downsampled1->size () << "/ "<<points1->size () <<"total points." << std::endl;
  std::cout << "downsample points2 :" << downsampled2->size () << "/ "<<points2->size () <<"total points." << std::endl;

  translation_only(downsampled2, downsampled1, points2, downsampled2_move, points2_move);

  // Compute surface normals
  const float normal_radius1 = 7.0;
  const float normal_radius2 = 5.0;
//  compute_surface_normals (downsampled1, normal_radius1, normals1);
//  compute_surface_normals (downsampled2, normal_radius2, normals2);
  compute_surface_normals (points1, normal_radius1, normals1);
  compute_surface_normals (points2_move, normal_radius2, normals2);


  // Compute SHOT features
  const float feature_radius1 = 7.0;
  const float feature_radius2 = 5.0;

  std::cout << "get ready !!!!!" << std::endl;

  compute_SHOT_features (points1, downsampled1, normals1, feature_radius1, descriptors_shot1);
  compute_SHOT_features (points2_move, downsampled2_move, normals2, feature_radius2, descriptors_shot2);

  std::cout << "compute feutures successfully" << std::endl;

//  find_feature_SHOT_correspondences (descriptors_shot1, descriptors_shot2, correspondences_all);
  find_feature_SHOT_correspondences (descriptors_shot2, descriptors_shot1, correspondences_all);

  rf_calculation(points1, downsampled1, normals1, scene_rf);
  rf_calculation(points2_move, downsampled2_move, normals2, model_rf);

  Hough_groupping(cg_size_, cg_thresh_, points2_move, points1, downsampled2_move, downsampled1, model_rf, scene_rf, correspondences_all, correspondences_hough, group_transform);
//  geometric_consistency(cg_size_, cg_thresh_, downsampled2, downsampled1, correspondences_all, correspondences_hough, group_transform);
 
//  estimation_feature_correspondences_SHOT(descriptors_shot1, descriptors_shot2, correspondences_all);
//  rejection_correspondences(downsampled1, downsampled2, correspondences_all, correspondences_inliers);

//  std::cout << "find correspondences successfully" << std::endl;

  // Visualize the two point clouds and their feature correspondences
//  visualize_correspondences (downsampled1, downsampled2, correspondences_inliers);
  static double last = pcl::getTime();
  std::cerr << "Using time : " << (last-first) << std::endl;
  visualize_correspondences (downsampled2_move, downsampled1, correspondences_hough);

//  calculate_transformation (downsampled2_move, downsampled1, correspondences_hough, downsampled2_trans);

  pcl::transformPointCloud (*downsampled2_move, *downsampled2_trans, group_transform);

//  calculate_transformation (downsampled1, downsampled2, correspondences_inliers, downsampled1_trans);

  // Visualize the superposition of point clouds and the another transformed point clouds 
  visualize_transformation (downsampled1, downsampled2_trans);
/* 
  pcl::PointCloud<pcl::PointXYZ>::Ptr finalpoints (new pcl::PointCloud<pcl::PointXYZ>);
  Eigen::Matrix4f transform;
  finalpoints = runICP(downsampled2_trans, downsampled1, transform); 

  visualize_transformation (downsampled1, finalpoints);
*/

  return (0);
}

