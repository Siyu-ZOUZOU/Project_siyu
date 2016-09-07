#include <vector>
#include <iostream>
#include <Eigen/Core>
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/vfh.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/console/parse.h>
#include <pcl/ml/svm_wrapper.h>

int number_ (142);
pcl::SVMParam param_;
int nr_fold_(0);

void
showHelp (char *filename)
{
  std::cout << std::endl;
  std::cout << "***************************************************************************" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "*                creation of features dataset - Usage Guide               *" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "***************************************************************************" << std::endl << std::endl;
  std::cout << "Usage: " << filename << " [output_model_file(*.model)] [Options]" << std::endl << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "-number val:     the number of models from dataset to calulate the VFH feature from 0 to 142 (default :"<< number_ << ")"  << std::endl;
  printf(
  "-s svm_type : set type of SVM (default 0)\n"
  "	0 -- C-SVC		(multi-class classification)\n"
  "	1 -- nu-SVC		(multi-class classification)\n"
  "	2 -- one-class SVM\n"
  "	3 -- epsilon-SVR	(regression)\n"
  "	4 -- nu-SVR		(regression)\n"
  "-t kernel_type : set type of kernel function (default 2)\n"
  "	0 -- linear: u'*v\n"
  "	1 -- polynomial: (gamma*u'*v + coef0)^degree\n"
  "	2 -- radial basis function: exp(-gamma*|u-v|^2)\n"
  "	3 -- sigmoid: tanh(gamma*u'*v + coef0)\n"
  "	4 -- precomputed kernel (kernel values in training_set_file)\n"
  "-d degree : set degree in kernel function (default 3)\n"
  "-g gamma : set gamma in kernel function (default 1/num_features)\n"
  "-r coef0 : set coef0 in kernel function (default 0)\n"
  "-c cost : set the parameter C of C-SVC, epsilon-SVR, and nu-SVR (default 1)\n"
  "-n nu : set the parameter nu of nu-SVC, one-class SVM, and nu-SVR (default 0.5)\n"
  "-p epsilon : set the epsilon in loss function of epsilon-SVR (default 0.1)\n"
  "-m cachesize : set cache memory size in MB (default 100)\n"
  "-e epsilon : set tolerance of termination criterion (default 0.001)\n"
  "-h shrinking : whether to use the shrinking heuristics, 0 or 1 (default 1)\n"
  "-b probability_estimates : whether to train a SVC or SVR model for probability estimates, 0 or 1 (default 0)\n"
  "-v n: n-fold cross validation mode\n"
  );
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
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;

  // Use a KdTree to perform neighborhood searches
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

//  std::cout << "start compute VFH features descriptors" << std::endl;
  // Compute the features
  vfh_est.compute (*descriptors_out);
//  std::cout << "Compute VFH descriptors successfully" << std::endl;

}

/* Function : output the *.txt file for Libsvm use*/
void
write2file (pcl::PointCloud<pcl::VFHSignature308>::Ptr &descriptors_vfh, const int count)
{
  if(count ==0)
  {
    ofstream outFile( "VFH_svm_features.txt" );
    if ( !outFile ) 
    {
      printf("Error opening output file: %s!\n", "VFH_svm_features.txt");
      exit( 1 );
    }
    for (size_t j = 0; j < descriptors_vfh->size (); j++)
    {
      outFile << "0 ";
      for(int q=0; q< 308; q++)
      {
        outFile << (q+1) << ":"<<descriptors_vfh->at(j).histogram[q] << " ";
      }
    } 
    outFile << std::endl;
  }
  else
  {
    ofstream ofresult( "VFH_svm_features.txt", ios::app); 
    for (size_t k = 0; k < descriptors_vfh->size (); k++)
    {
      ofresult << count << " ";
      for(int p=0; p< 308; p++)
      {
        ofresult << (p+1) << ":"<<descriptors_vfh->at(k).histogram[p] << " ";
      }
    } 
    ofresult << std::endl;
    if(count == number_)   ofresult.close();
  }
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
parseCommandLine (int argc, char *argv[])
{
  //Show help
  if (( argc < 2 ) || pcl::console::find_switch (argc, argv, "-h"))
  {
    showHelp (argv[0]);
    exit (0);
  }

  //General parameters
  pcl::console::parse_argument (argc, argv, "-number", number_);
  pcl::console::parse_argument (argc, argv, "-s", param_.svm_type);
  pcl::console::parse_argument (argc, argv, "-t", param_.kernel_type);
  pcl::console::parse_argument (argc, argv, "-g", param_.gamma);
  pcl::console::parse_argument (argc, argv, "-r", param_.coef0);
  pcl::console::parse_argument (argc, argv, "-n", param_.nu);
  pcl::console::parse_argument (argc, argv, "-m", param_.cache_size);
  pcl::console::parse_argument (argc, argv, "-c", param_.C);
  pcl::console::parse_argument (argc, argv, "-e", param_.eps);
  pcl::console::parse_argument (argc, argv, "-p", param_.p);
  pcl::console::parse_argument (argc, argv, "-h", param_.shrinking);
  pcl::console::parse_argument (argc, argv, "-b", param_.probability);
  pcl::console::parse_argument (argc, argv, "-r", param_.coef0);
  pcl::console::parse_argument (argc, argv, "-n", param_.nu);
  pcl::console::parse_argument (argc, argv, "-v", nr_fold_);
  if(nr_fold_ == 1 && nr_fold_ == 2)
  {
    fprintf(stderr,"n-fold cross validation: n must >= 2\n");
    showHelp (argv[0]);
    exit (0);
  }
}

int main (int argc, char *argv[])
{
  const char *modelFileName = argv[1];
  parseCommandLine (argc, argv);

  std::vector<pcl::SVMData> tranning_dataset;
  for(int i=0; i< (number_+ 1) ; i++)
  {
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

    std::cout<<"write features of model No."<<i<<std::endl;
    // Store the VF histogram into the txt for libsvm input
    write2svmdata(descriptors_vfh1, i, tranning_dataset);
  }
  std::cout<<"All the SVM datas have been created"<<std::endl;

  // SVM trainning
  pcl::SVMTrain Mytrainner;
  Mytrainner.setParameters(param_);
  Mytrainner.resetTrainingSet ();
  Mytrainner.setInputTrainingSet (tranning_dataset);
  Mytrainner.trainClassifier ();
  Mytrainner.saveClassifierModel(modelFileName);
  std::cout<<"Output trainning model : "<< modelFileName <<std::endl;
  return (0);
}
