#define PCL_NO_PRECOMPILE
#include <vector>
#include <iostream>
#include <Eigen/Core>
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include <pcl/correspondence.h>
#include "pcl/io/pcd_io.h"
#include <pcl/search/kdtree.h>
#include "pcl/features/normal_3d_omp.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/transformation_estimation_svd_scale.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/esf.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/features/rops_estimation.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types_conversion.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/pcl_macros.h>
#include <pcl/register_point_struct.h>
#include <pcl/console/parse.h>

std::string model_filename_;
std::string scene_filename_;
std::string room_filename_;

void
showHelp (char *filename)
{
  std::cout << std::endl;
  std::cout << "***************************************************************************" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "*             Correspondence Grouping Tutorial - Usage Guide              *" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "***************************************************************************" << std::endl << std::endl;
  std::cout << "Usage: "  << filename  << " <scene_filename.pcd> <model_filename.pcd> <model_room_filename.pcd>" << std::endl << std::endl;
  std::cout << "Example: " << filename << " method1_seg.pcd  maquette.pcd  chambre.pcd" << std::endl << std::endl;
}


struct ROPS135
{
  float histogram[135];
  static int descriptorSize () { return 135; }
  friend std::ostream& operator << (std::ostream& os, const ROPS135& p);
};
PCL_EXPORTS std::ostream& operator << (std::ostream& os, const ROPS135& p);

POINT_CLOUD_REGISTER_POINT_STRUCT (ROPS135,
    (float[135], histogram, rops)
)


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
compute_ROPS_features (pcl::PointCloud<pcl::PointXYZ>::Ptr &points,
                       pcl::PointCloud<pcl::Normal>::Ptr &normals,
                       float feature_radius,
                       pcl::PointCloud<ROPS135>::Ptr &descriptors_out)
{
// Perform triangulation.
  pcl::PointCloud<pcl::PointNormal>::Ptr cloudNormals(new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields(*points, *normals, *cloudNormals);
  pcl::search::KdTree<pcl::PointNormal>::Ptr kdtree2(new pcl::search::KdTree<pcl::PointNormal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);

  kdtree2->setInputCloud(cloudNormals);
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> triangulation;
  pcl::PolygonMesh triangles;
  triangulation.setSearchRadius(feature_radius);
  triangulation.setMu(2.5);
  triangulation.setMaximumNearestNeighbors(100);
  triangulation.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees.
  triangulation.setNormalConsistency(false);
  triangulation.setMinimumAngle(M_PI / 18); // 10 degrees.
  triangulation.setMaximumAngle(2 * M_PI / 3); // 120 degrees.
  triangulation.setInputCloud(cloudNormals);
  triangulation.setSearchMethod(kdtree2);
  triangulation.reconstruct(triangles);
 
  // Note: only compute descriptors for chosen keypoints. It has been omitted here for simplicity.
  // RoPs estimation object.
  pcl::ROPSEstimation<pcl::PointXYZ, ROPS135> rops;
  rops.setInputCloud(points);
  rops.setSearchMethod(kdtree);
  rops.setRadiusSearch(0.025);
  rops.setTriangles(triangles.polygons);
  // Number of partition bins that is used for distribution matrix calculation.
  rops.setNumberOfPartitionBins(5);
  // The greater the number of rotations is, the bigger the resulting descriptor.
  // Make sure to change the histogram size accordingly.
  rops.setNumberOfRotations(3);
  // Support radius that is used to crop the local surface of the point.
  rops.setSupportRadius(feature_radius);
  rops.compute(*descriptors_out);

}


void estimation_feature_correspondences_ROPS(pcl::PointCloud<ROPS135>::Ptr &source_descriptors,
                                             pcl::PointCloud<ROPS135>::Ptr &target_descriptors, 
                                             pcl::CorrespondencesPtr &correspondences_all)
{
  PCL_INFO("Finding correspondences...\n");
  pcl::registration::CorrespondenceEstimation<ROPS135, ROPS135> ce;
  ce.setInputSource (source_descriptors);
  ce.setInputTarget (target_descriptors);
  ce.determineReciprocalCorrespondences (*correspondences_all);
  PCL_INFO("%d correspondences found\n", correspondences_all->size ());
}


void
find_feature_ROPS_correspondences (pcl::PointCloud<ROPS135>::Ptr &source_descriptors,
                                   pcl::PointCloud<ROPS135>::Ptr &target_descriptors,
                                   pcl::CorrespondencesPtr &correspondences_all)
{
  pcl::search::KdTree<ROPS135> matching;
  matching.setInputCloud(target_descriptors);
  
  // A Correspondence object stores the indices of the query and the match,
  // and the distance/weight.
 
  // Check every descriptor computed for the scene.
  for (size_t i = 0; i < source_descriptors->size(); ++i)
  {
    std::vector<int> neighbors(1);
    std::vector<float> squaredDistances(1);

    // Ignore NaNs.
    if (!pcl_isfinite (source_descriptors->at (i).histogram[0])) //skipping NaNs
    {
      continue;
    }
    // Find the nearest neighbor (in descriptor space)...
    int neighborCount = matching.nearestKSearch(source_descriptors->at(i), 1, neighbors, squaredDistances);
    // ...and add a new correspondence if the distance is less than a threshold
    // (ROPS distances are between 0 and 1, other descriptors use different metrics).
    pcl::Correspondence correspondence(static_cast<int>(i), neighbors[0], squaredDistances[0]);
    correspondences_all->push_back(correspondence);

  }
  std::cout << "Found " << correspondences_all->size() << " ROPS correspondences." << std::endl;
}


void
compute_ESF_features (pcl::PointCloud<pcl::PointXYZ>::Ptr &points,
                      pcl::PointCloud<pcl::ESFSignature640>::Ptr &descriptors_out)
{
  // Create a PFHEstimation object
  pcl::ESFEstimation<pcl::PointXYZ, pcl::ESFSignature640> esf_est;

  // Set the input points and surface normals
  esf_est.setInputCloud (points);
  std::cout << "start compute ESF features descriptors" << std::endl;
  // Compute the features
  esf_est.compute (*descriptors_out);
  std::cout << "Compute ESF descriptors successfully" << std::endl;
/*
  pcl::visualization::PCLPlotter plotter;
  // need to set the size of the descriptor beforehand.
  plotter.addFeatureHistogram(*descriptors_out, 640); 
  plotter.plot();
*/
}


void estimation_feature_correspondences(pcl::PointCloud<pcl::PFHSignature125>::Ptr &source_descriptors,
                              pcl::PointCloud<pcl::PFHSignature125>::Ptr &target_descriptors, pcl::CorrespondencesPtr &correspondences_all)
{
  PCL_INFO("Finding correspondences...\n");
  pcl::registration::CorrespondenceEstimation<pcl::PFHSignature125, pcl::PFHSignature125> ce;
  ce.setInputSource (source_descriptors);
  ce.setInputTarget (target_descriptors);
  ce.determineReciprocalCorrespondences (*correspondences_all);
  PCL_INFO("%d correspondences found\n", correspondences_all->size ());
}


void rejection_correspondences(pcl::PointCloud<pcl::PointXYZ>::Ptr &points1, pcl::PointCloud<pcl::PointXYZ>::Ptr &points2, int num_iteration, 
                               pcl::CorrespondencesPtr &correspondences_all, pcl::CorrespondencesPtr &correspondences_inliers)
{

  pcl::CorrespondencesPtr correspondences_x (new pcl::Correspondences);
  pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> sac;
  sac.setInputSource (points1);
  sac.setInputTarget (points2);
  sac.setInlierThreshold (0.1);
  sac.setMaximumIterations (num_iteration);
  sac.setInputCorrespondences (correspondences_all);
  sac.getCorrespondences (*correspondences_x);

  for (size_t i=0; i < correspondences_x -> size (); i++){
    if (correspondences_x ->at(i).distance < 1.5e-07) {
      correspondences_inliers->push_back(correspondences_x->at(i));
    }
  }
  PCL_INFO("%d inliers correspondences\n", correspondences_inliers->size ());
/*
  for(size_t i =0; i< correspondences_inliers->size ();i++){
    std::cerr<<"the linliers correspondences"<< correspondences_inliers->at(i)<<std::endl;
  }
*/
}


void
find_feature_ESF_correspondences (pcl::PointCloud<pcl::ESFSignature640>::Ptr &source_descriptors,
                                  pcl::PointCloud<pcl::ESFSignature640>::Ptr &target_descriptors,
                                  pcl::CorrespondencesPtr &correspondences_all)
{

  // Use a KdTree to search for the nearest matches in feature space
  pcl::search::KdTree<pcl::ESFSignature640> descriptor_kdtree;
  descriptor_kdtree.setInputCloud (target_descriptors);
  PCL_INFO("source_descriptors' number is %d \n", source_descriptors->size ());
  PCL_INFO("target_descriptors' number is %d \n", target_descriptors->size ());
  // Find the index of the best match for each point, and store it in "correspondences_out"
 
  for (size_t i = 0; i < source_descriptors->size (); ++i)
  {
    std::vector<int> neigh_indices (1);
    std::vector<float> neigh_sqr_dists (1);

    if (!pcl_isfinite (source_descriptors->at (i).histogram[0])) //skipping NaNs
    {
      continue;
    }

    int found_neighs = descriptor_kdtree.nearestKSearch (source_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
    pcl::Correspondence corr ( static_cast<int> (i), neigh_indices[0], neigh_sqr_dists[0]);
    correspondences_all->push_back (corr);
    std::cout<<corr<<std::endl;

  }
  PCL_INFO("correspondences found %d \n", correspondences_all->size ());
}


void calculate_transformation (pcl::PointCloud<pcl::PointXYZ>::Ptr &points1,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr &points2,
                               pcl::CorrespondencesPtr &correspondences_inliers,
                               Eigen::Matrix4f &transform)
{
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
  transform = transformation2;
}


void calculate_transformation_clusters (pcl::PointCloud<pcl::PointXYZ>::Ptr &points1,
                                	pcl::PointCloud<pcl::PointXYZ>::Ptr &points2,
                              		pcl::PointCloud<pcl::PointXYZ>::Ptr &points_all, 
                                	pcl::CorrespondencesPtr &correspondences_inliers,
                                        Eigen::Matrix4f& transform,
                                	pcl::PointCloud<pcl::PointXYZ>::Ptr &points_trans)
{

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
  pcl::transformPointCloud (*points_all, *points_trans, transformation2);
  transform = transformation2;
}


void alignement (pcl::PointCloud<pcl::PointXYZ>::Ptr &points1,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr &points2,
                 pcl::PointCloud<ROPS135>::Ptr &points1_descriptors,
                 pcl::PointCloud<ROPS135>::Ptr &points2_descriptors,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr &object_aligned )
{
  const float leaf = 0.005f;
  pcl::console::print_highlight ("Starting alignment...\n");
  pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, ROPS135> align;
  align.setInputSource (points1);
  align.setSourceFeatures (points1_descriptors);
  align.setInputTarget (points2);
  align.setTargetFeatures (points2_descriptors);
  align.setMaximumIterations (70000); // Number of RANSAC iterations
  align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness (15); // Number of nearest features to use
  align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
  align.setInlierFraction (0.3f); // Required inlier fraction for accepting a pose hypothesis
  align.align (*object_aligned);
  
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
    visu.addPointCloud (points2, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> (points2, 255.0, 255.0, 0.0), "model");
    visu.addPointCloud (object_aligned, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> (object_aligned, 255.0, 0.0, 0.0), "object_aligned");
    visu.spin ();
  }
  else
  {
    pcl::console::print_error ("Alignment failed!\n");
  }
}


pcl::PointCloud<pcl::PointXYZ>::Ptr runICP(pcl::PointCloud<pcl::PointXYZ>::Ptr irCloud, 
                                           pcl::PointCloud<pcl::PointXYZ>::Ptr eoCloud, 
                                           Eigen::Matrix4f& transform) 
{
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  // Set the input source and target
  icp.setInputSource(irCloud);
  icp.setInputTarget(eoCloud);

  // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
  icp.setMaxCorrespondenceDistance (0.1);
  // Set the maximum number of iterations (criterion 1)
  icp.setMaximumIterations (500);
  // Set the transformation epsilon (criterion 2)
  // icp.setTransformationEpsilon (1e-16);//16
  // Set the euclidean distance difference epsilon (criterion 3)
  // icp.setEuclideanFitnessEpsilon (.5);

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


void visualize_normals (const pcl::PointCloud<pcl::PointXYZ>::Ptr points,
                        const pcl::PointCloud<pcl::PointXYZ>::Ptr normal_points,
                        const pcl::PointCloud<pcl::Normal>::Ptr normals)
{
  // Add the points and normals to the vizualizer
  pcl::visualization::PCLVisualizer viz;
  viz.addPointCloud (points, "points");
  viz.addPointCloud (normal_points, "normal_points");

  viz.addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (normal_points, normals, 1, 0.01, "normals");

  // Give control over to the visualizer
  viz.spin ();
}


void visualize_correspondences (const pcl::PointCloud<pcl::PointXYZ>::Ptr points1,
                                const pcl::PointCloud<pcl::PointXYZ>::Ptr points2,
                                const pcl::CorrespondencesPtr &correspondences_all)
{
  // Create some new point clouds to hold our transformed data
  pcl::PointCloud<pcl::PointXYZ>::Ptr points_left (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr points_right (new pcl::PointCloud<pcl::PointXYZ>);

  // Shift the first clouds' points to the left
  //const Eigen::Vector3f translate (0.0, 0.0, 0.3);
  const Eigen::Vector3f translate (0.0, 0.5, 0.0);
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

    viz.addLine<pcl::PointXYZ, pcl::PointXYZ> (p_left, p_right, r, g, b, ss.str ());
  }

  // Give control over to the visualizer
  viz.spin ();
}

void visualize_correspondences_clusters (const pcl::PointCloud<pcl::PointXYZ>::Ptr points1,
                               		 const pcl::PointCloud<pcl::PointXYZ>::Ptr points2,
                               		 const pcl::PointCloud<pcl::PointXYZ>::Ptr points_all,
                                	 const pcl::CorrespondencesPtr &correspondences_all)
{
  // Create some new point clouds to hold our transformed data
  pcl::PointCloud<pcl::PointXYZ>::Ptr points_left (new pcl::PointCloud<pcl::PointXYZ>), points_left_ (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr points_right (new pcl::PointCloud<pcl::PointXYZ>);

  // Shift the first clouds' points to the left
  //const Eigen::Vector3f translate (0.0, 0.0, 0.3);
  const Eigen::Vector3f translate (0.0, 0.00005, 0.0);
  const Eigen::Quaternionf no_rotation (0, 0, 0, 0);

  pcl::transformPointCloud (*points1, *points_left, translate, no_rotation);
  pcl::transformPointCloud (*points_all, *points_left_, translate, no_rotation);

  // Shift the second clouds' points to the right
  pcl::transformPointCloud (*points2, *points_right, -translate, no_rotation);

  // Add the clouds to the vizualizer
  pcl::visualization::PCLVisualizer viz;
  viz.addPointCloud (points_left, "points_left");
  viz.addPointCloud (points_left_, "points_left_");
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
  viz.addCoordinateSystem (1.0, "cloud", 0);
  viz.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "points_left");
  viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "points_right");

  // Give control over to the visualizer
  viz.spin ();
}

void visualize_transformation(const pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
                              const pcl::PointCloud<pcl::PointXYZ>::Ptr viewpoint,
                              const pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud)
{

  printf(  "\nPoint cloud colors :  white  = original point cloud\n"
      "                        red  = transformed point cloud\n");

  pcl::visualization::PCLVisualizer viewer ("Matrix transformation");

   // Define R,G,B colors for the point cloud
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (source_cloud, 255, 255, 255);
  // We add the point cloud to the viewer and pass the color handler
  viewer.addPointCloud (source_cloud, source_cloud_color_handler, "original_cloud");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (transformed_cloud, 230, 20, 20); // Red
  viewer.addPointCloud (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> viewpoint_color_handler (viewpoint, 0.0, 255, 255.0); //green?
  viewer.addPointCloud (viewpoint, viewpoint_color_handler, "transformed_viewpoint");

 viewer.addCoordinateSystem (1.0, "cloud", 0);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_viewpoint");
  //viewer.setPosition(800, 400); // Setting visualiser window position

  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce ();
  }
}

void cluster_euclidean(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, 
                       std::vector<pcl::PointIndices> &cluster_indices)
{
	 
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.05); // 2cm
  ec.setMinClusterSize (500);
  ec.setMaxClusterSize (50000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);
  std::cout << "totally  " << cluster_indices.size() << " clusters " << std::endl;
}

void transform_viewpoint (pcl::PointCloud<pcl::PointXYZ>::Ptr &source_cloud,
                          Eigen::Matrix4f& transform, 
                          pcl::PointCloud<pcl::PointXYZ>::Ptr &transformed_cloud)
{
  // Generate the data
  for (float y=-0.01f; y<=0.03f; y+=0.001f) {
    for (float z=-0.01f; z<=0.03f; z+=0.001f) {
      pcl::PointXYZ point;
      point.x = 0.01f - y;
      point.y = y;
      point.z = z;
      source_cloud->points.push_back(point);
    }
  }
  source_cloud->width = (uint32_t) source_cloud->points.size();
  source_cloud->height = 1;

  pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform);
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
  if (filenames.size () != 3)
  {
    std::cout << "Filenames missing.\n";
    showHelp (argv[0]);
    exit (-1);
  }
  scene_filename_ = argv[filenames[0]];
  model_filename_ = argv[filenames[1]];
  room_filename_  = argv[filenames[2]];
}


int main (int argc, char *argv[])
{

  parseCommandLine (argc, argv);

  // Create some new point clouds to hold our data
  pcl::PointCloud<pcl::PointXYZ>::Ptr points1 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled1 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr normals_cluster1 (new pcl::PointCloud<pcl::Normal>);

  pcl::PointCloud<pcl::ESFSignature640>::Ptr descriptors_esf1 (new pcl::PointCloud<pcl::ESFSignature640>);
  pcl::PointCloud<ROPS135>::Ptr descriptors_rops1 (new pcl::PointCloud<ROPS135>);

  pcl::PointCloud<pcl::PointXYZ>::Ptr points2 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled2 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::ESFSignature640>::Ptr descriptors_esf2 (new pcl::PointCloud<pcl::ESFSignature640>);
  pcl::PointCloud<ROPS135>::Ptr descriptors_rops2 (new pcl::PointCloud<ROPS135>);

  pcl::PointCloud<pcl::PointXYZ>::Ptr points3 (new pcl::PointCloud<pcl::PointXYZ>);
 
  pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud_all (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr object_aligned (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled1_trans (new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::CorrespondencesPtr correspondences_all (new pcl::Correspondences);
  pcl::CorrespondencesPtr correspondences_inliers (new pcl::Correspondences);
  pcl::CorrespondencesPtr correspondences_max (new pcl::Correspondences);
  Eigen::Matrix4f transform_total;
  pcl::PointCloud<pcl::PointXYZ>::Ptr viewpoint_ (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr viewpoint_ori (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr viewpoint_refine (new pcl::PointCloud<pcl::PointXYZ>());

  //  Load clouds
  if (pcl::io::loadPCDFile (scene_filename_, *points1) < 0)
  {
    std::cout << "Error loading model cloud." << std::endl;
    showHelp (argv[0]);
    return (-1);
  }
  if (pcl::io::loadPCDFile (model_filename_, *points2) < 0)
  {
    std::cout << "Error loading scene cloud." << std::endl;
    showHelp (argv[0]);
    return (-1);
  }
  if (pcl::io::loadPCDFile (room_filename_, *points3) < 0)
  {
    std::cout << "Error loading model cloud." << std::endl;
    showHelp (argv[0]);
    return (-1);
  }
  std::cout << "load point clouds successfully"  << std::endl;

  std::cout << "Clustering starts !!!!!" << std::endl;

  cluster_euclidean(points1, cluster_indices);
  const float voxel_grid_leaf_size2 = 0.02;
  downsample (points2, voxel_grid_leaf_size2, downsampled2);
  compute_ESF_features (downsampled2, descriptors_esf2);

  float cor_size_i = 50000.0;
  float cor_size_m = 0.0;
  int i = 0;
  int index_max = 0;
  int restraint =0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) 
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr points_clustered (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) 
    		points_clustered->points.push_back (points1->points[*pit]);

    pcl::CorrespondencesPtr correspondences_ (new pcl::Correspondences);
    std::cout << "index" << i << "point cloud"<< std::endl;
    const float voxel_grid_leaf_size1 = 0.02;
    downsample (points_clustered, voxel_grid_leaf_size1, downsampled1);

    std::cout << "downsample points1 :" << downsampled1->size () << " total points." << std::endl;
    std::cout << "downsample points2 :" << downsampled2->size () << " total points." << std::endl;

    *clustered_cloud_all += *downsampled1;

    // Compute ESF features
    compute_ESF_features (downsampled1, descriptors_esf1);

    // Find feature correspondences
    find_feature_ESF_correspondences (descriptors_esf1, descriptors_esf2, correspondences_);

    cor_size_m = correspondences_->at(0).distance;
    
    if (cor_size_m < cor_size_i || cor_size_m == 0.0 )
    {
      cor_size_i = cor_size_m;
      index_max = i;
      *clustered_cloud = *downsampled1;
      *correspondences_max = *correspondences_;
    }
    i++;
  }

  std::cout << "Maximum number of inlier correspondences is " << cor_size_i << ". Index of max correspondence is " << index_max <<std::endl;

  const float normal_radius1 = 0.025;
  compute_surface_normals (clustered_cloud, normal_radius1, normals_cluster1);

  const float feature_radius1 = 0.025;
  compute_ROPS_features (clustered_cloud, normals_cluster1, feature_radius1, descriptors_rops1);
  pcl::io::loadPCDFile ("maquette_rops_features.pcd", *descriptors_rops2);
  estimation_feature_correspondences_ROPS(descriptors_rops1, descriptors_rops2, correspondences_all);
  int num_iteration = 800;
  rejection_correspondences(clustered_cloud, downsampled2, num_iteration, correspondences_all, correspondences_inliers);
  std::cout << "find correspondences successfully" << std::endl;

  // Visualize the two point clouds and their feature correspondences
  // visualize_correspondences_clusters (clustered_cloud, downsampled2, clustered_cloud_all, correspondences_inliers); 

  calculate_transformation (clustered_cloud, downsampled2, correspondences_inliers, transform_total);

//  while (transform_total(1 , 1) < 0.8 && restraint < 10)
//  {
    std::cerr<<"cannot match correctly, but we try one more time"<<std::endl;
    num_iteration+= 500;
    rejection_correspondences(clustered_cloud, downsampled2, num_iteration, correspondences_all, correspondences_inliers);
    calculate_transformation (clustered_cloud, downsampled2, correspondences_inliers, transform_total);
    pcl::transformPointCloud (*clustered_cloud, *downsampled1_trans, transform_total);
    restraint ++;
//  }
//  if(transform_total(1 , 1) < 0.8){

//    std::cerr<<"cannot match correctly, we have tried 10 times, get out of programme."<<std::endl;

//  }else
//  {
    pcl::transformPointCloud (*clustered_cloud, *downsampled1_trans, transform_total);
    transform_viewpoint(viewpoint_ori, transform_total, viewpoint_);
 
    std::cerr<<"ICP STARTS !"<<std::endl;
  //  alignement (clustered_cloud, downsampled2, descriptors_rops1, descriptors_rops2, object_aligned);
    pcl::PointCloud<pcl::PointXYZ>::Ptr finalpoints (new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix4f transform;
    finalpoints = runICP(downsampled1_trans, downsampled2, transform); 


    std::cerr<<"ICP FINISHED !"<<std::endl;
    pcl::transformPointCloud (*viewpoint_, *viewpoint_refine, transform);
    // Visualize the superposition of point clouds and the another transformed point clouds
    visualize_transformation (points3, viewpoint_,  finalpoints);
//  }
  return (0);
}

