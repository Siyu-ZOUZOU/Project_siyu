#define PCL_NO_PRECOMPILE
#include <vector>
#include <iostream>
#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/transforms.h>
#include <pcl/pcl_macros.h>
#include <pcl/console/parse.h>

int angle_ (30);
float division_ (6.0f);
float size_ (1.0f);
int name_number_ (0);

std::string model_filename_;

void
showHelp (char *filename)
{
  std::cout << std::endl;
  std::cout << "***************************************************************************" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "*             Dataset creation - Usage Guide              *" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "***************************************************************************" << std::endl << std::endl;
  std::cout << "Usage: " << filename << " model_filename.pcd [Options]" << std::endl << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "     --angle val:          rotation angle (degree) (default :"<< angle_ << ")"  << std::endl;
  std::cout << "       every 15 from -90 to 90, for example: -90, -75, -60, -45, etc."  << std::endl;
  std::cout << "     --size val:               size (default :" << size_ << ")" << std::endl;
  std::cout << "       it's better from 0.5 to -2.0 "  << std::endl;
  std::cout << "     --name_number val:        name_number (default :" << name_number_ << ")" << std::endl;
  std::cout << "       The name is : training_model_[name_number].pcd"  << std::endl;
}


void transform_dataset (pcl::PointCloud<pcl::PointXYZ>::Ptr &points1,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr &points1_trans,
                        float & division_,
                        float & size_,
                        int & name_number_) 
{
  // to rotate the model grounded //
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_ (new pcl::PointCloud<pcl::PointXYZ> ());  
  Eigen::Affine3f transform_1 = Eigen::Affine3f::Identity();

  Eigen::Matrix3f angle;
  angle = Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitX())
          * Eigen::AngleAxisf(M_PI,  Eigen::Vector3f::UnitY())
          * Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitZ());

  transform_1.translation() << 108, 234, 2105;
  transform_1.rotate (angle);
  transform_1.scale (150.0);
  pcl::transformPointCloud (*points1, *transformed_cloud_, transform_1);
  
  ////// do not modify above //////

  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

  Eigen::Matrix3f angle_set;
  angle_set = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX())
              * Eigen::AngleAxisf(M_PI/division_,  Eigen::Vector3f::UnitY())
              * Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ());


  transform_2.rotate (angle_set);
  transform_2.scale (size_);
  pcl::transformPointCloud (*transformed_cloud_, *points1_trans, transform_2);

  std::stringstream ss1;
  ss1 << "training_model_"<<name_number_<<".pcd";
  
  pcl::io::savePCDFileASCII(ss1.str (), *points1_trans);
  std::cout << "save No."<<name_number_<< " to the PCD."<< std::endl;
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
  if (filenames.size () != 1)
  {
    std::cout << "Filenames missing.\n";
    showHelp (argv[0]);
    exit (-1);
  }

  model_filename_ = argv[filenames[0]];

  //General parameters
  pcl::console::parse_argument (argc, argv, "--angle", angle_);
  pcl::console::parse_argument (argc, argv, "--size", size_);
  pcl::console::parse_argument (argc, argv, "--name_number", name_number_);

  if (angle_ == -90 )  division_ = 2;
  else if (angle_ == -75 )  division_ = 2.4;
  else if (angle_ == -60 )  division_ = 3;
  else if (angle_ == -45 )  division_ = 4;
  else if (angle_ == -30 )  division_ = 6;
  else if (angle_ == -15 )  division_ = 12;
  else if (angle_ == 0 )  division_ = 10000;
  else if (angle_ == 15 )  division_ = -12;
  else if (angle_ == 30 )  division_ = -6;
  else if (angle_ == 45 )  division_ = -4;
  else if (angle_ == 60 )  division_ = -3;
  else if (angle_ == 75 )  division_ = -2.4;
  else if (angle_ == 90 )  division_ = -2;
  else division_ = 10000;
}

                               		 
int main (int argc, char *argv[])
{

  parseCommandLine (argc, argv);
  // Create some new point clouds to hold our data
  pcl::PointCloud<pcl::PointXYZ>::Ptr points1 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr points1_move (new pcl::PointCloud<pcl::PointXYZ>);

  //  Load clouds
  if (pcl::io::loadPCDFile (model_filename_, *points1) < 0)
  {
    std::cout << "Error loading model cloud." << std::endl;
    showHelp (argv[0]);
    return (-1);
  }

  std::cout << "load " << model_filename_ << " successfully" << std::endl;
  transform_dataset(points1, points1_move, division_, size_, name_number_);

  return (0);
}
