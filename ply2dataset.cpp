/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/transforms.h>
#include <vtkVersion.h>
#include <vtkPLYReader.h>
#include <vtkOBJReader.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataMapper.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>

inline double
uniform_deviate (int seed)
{
  double ran = seed * (1.0 / (RAND_MAX + 1.0));
  return ran;
}

inline void
randomPointTriangle (float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2, float c3,
                     Eigen::Vector4f& p)
{
  float r1 = static_cast<float> (uniform_deviate (rand ()));
  float r2 = static_cast<float> (uniform_deviate (rand ()));
  float r1sqr = sqrtf (r1);
  float OneMinR1Sqr = (1 - r1sqr);
  float OneMinR2 = (1 - r2);
  a1 *= OneMinR1Sqr;
  a2 *= OneMinR1Sqr;
  a3 *= OneMinR1Sqr;
  b1 *= OneMinR2;
  b2 *= OneMinR2;
  b3 *= OneMinR2;
  c1 = r1sqr * (r2 * c1 + b1) + a1;
  c2 = r1sqr * (r2 * c2 + b2) + a2;
  c3 = r1sqr * (r2 * c3 + b3) + a3;
  p[0] = c1;
  p[1] = c2;
  p[2] = c3;
  p[3] = 0;
}

inline void
randPSurface (vtkPolyData * polydata, std::vector<double> * cumulativeAreas, double totalArea, Eigen::Vector4f& p)
{
  float r = static_cast<float> (uniform_deviate (rand ()) * totalArea);

  std::vector<double>::iterator low = std::lower_bound (cumulativeAreas->begin (), cumulativeAreas->end (), r);
  vtkIdType el = vtkIdType (low - cumulativeAreas->begin ());

  double A[3], B[3], C[3];
  vtkIdType npts = 0;
  vtkIdType *ptIds = NULL;
  polydata->GetCellPoints (el, npts, ptIds);
  polydata->GetPoint (ptIds[0], A);
  polydata->GetPoint (ptIds[1], B);
  polydata->GetPoint (ptIds[2], C);
  randomPointTriangle (float (A[0]), float (A[1]), float (A[2]), 
                       float (B[0]), float (B[1]), float (B[2]), 
                       float (C[0]), float (C[1]), float (C[2]), p);
}

void
uniform_sampling (vtkSmartPointer<vtkPolyData> polydata, size_t n_samples, pcl::PointCloud<pcl::PointXYZ> & cloud_out)
{
  polydata->BuildCells ();
  vtkSmartPointer<vtkCellArray> cells = polydata->GetPolys ();

  double p1[3], p2[3], p3[3], totalArea = 0;
  std::vector<double> cumulativeAreas (cells->GetNumberOfCells (), 0);
  size_t i = 0;
  vtkIdType npts = 0, *ptIds = NULL;
  for (cells->InitTraversal (); cells->GetNextCell (npts, ptIds); i++)
  {
    polydata->GetPoint (ptIds[0], p1);
    polydata->GetPoint (ptIds[1], p2);
    polydata->GetPoint (ptIds[2], p3);
    totalArea += vtkTriangle::TriangleArea (p1, p2, p3);
    cumulativeAreas[i] = totalArea;
  }

  cloud_out.points.resize (n_samples);
  cloud_out.width = static_cast<pcl::uint32_t> (n_samples);
  cloud_out.height = 1;

  for (i = 0; i < n_samples; i++)
  {
    Eigen::Vector4f p;
    randPSurface (polydata, &cumulativeAreas, totalArea, p);
    cloud_out.points[i].x = p[0];
    cloud_out.points[i].y = p[1];
    cloud_out.points[i].z = p[2];
  }
}

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

int SAMPLE_POINTS_ = 100000;
float leaf_size = 0.005f;
int angle_ (30);
float division_ (6.0f);
float size_ (1.0f);
int name_number_ (0);

std::vector<int> ply_file_indices;
std::vector<int> obj_file_indices;

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
  transformPointCloud (*points1, *transformed_cloud_, transform_1);
  
  ////// do not modify above //////

  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

  Eigen::Matrix3f angle_set;
  angle_set = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX())
              * Eigen::AngleAxisf(M_PI/division_,  Eigen::Vector3f::UnitY())
              * Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ());


  transform_2.rotate (angle_set);
  transform_2.scale (size_);
  transformPointCloud (*transformed_cloud_, *points1_trans, transform_2);

  std::stringstream ss1;
  ss1 << "training_model_"<<name_number_<<".pcd";
  
  savePCDFileASCII(ss1.str (), *points1_trans);
  std::cout << "save No."<<name_number_<< " to the PCD."<< std::endl;
}

void
showHelp (char *filename)
{

  std::cout << std::endl;
  std::cout << "***************************************************************************" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "*             Dataset creation - Usage Guide              *" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "***************************************************************************" << std::endl << std::endl;
  std::cout << "Usage: " << filename << " input.{ply,obj} [Options]" << std::endl << std::endl;
  std::cout << "Options:" << std::endl;
  print_info ("      --n_samples val      = number of samples (default: ");
  print_value ("%d", SAMPLE_POINTS_);
  print_info (")\n");
  print_info ("      --leaf_size val      = the XYZ leaf size for the VoxelGrid -- for data reduction (default: ");
  print_value ("%f", leaf_size);
  print_info (" m)\n");
  std::cout << "     --angle val:         = rotation angle (degree) (default :"<< angle_ << ")"  << std::endl;
  std::cout << "       every 15 from -90 to 90, for example: -90, -75, -60, -45, etc."  << std::endl;
  std::cout << "     --size val:          = size (default :" << size_ << ")" << std::endl;
  std::cout << "       it's better from 0.5 to -2.0 "  << std::endl;
  std::cout << "     --name_number val:   = name_number (default :" << name_number_ << ")" << std::endl;
  std::cout << "       The output name is : training_model_[name_number].pcd"  << std::endl;

}

void
parseCommandLine (int argc, char *argv[])
{

  print_info ("Convert a CAD model to a point cloud using uniform sampling. For more information, use: %s -h\n",
              argv[0]);

  //Show help
  if (pcl::console::find_switch (argc, argv, "-h"))
  {
    showHelp (argv[0]);
    exit (0);
  }

  // Parse command line arguments
  parse_argument (argc, argv, "-n_samples", SAMPLE_POINTS_);
  parse_argument (argc, argv, "-leaf_size", leaf_size);
  parse_argument (argc, argv, "--angle", angle_);
  parse_argument (argc, argv, "--size", size_);
  parse_argument (argc, argv, "--name_number", name_number_);

  // Parse the command line arguments for .ply files
  ply_file_indices = parse_file_extension_argument (argc, argv, ".ply");
  obj_file_indices = parse_file_extension_argument (argc, argv, ".obj");
  if (ply_file_indices.size () != 1 && obj_file_indices.size () != 1)
  {
    print_error ("Need a single input PLY/OBJ file to continue.\n");
    showHelp (argv[0]);
    exit (0);
  }

  //General parameters

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


/* ---[ */
int
main (int argc, char **argv)
{

  parseCommandLine (argc, argv);

  vtkSmartPointer<vtkPolyData> polydata1 = vtkSmartPointer<vtkPolyData>::New ();
  if (ply_file_indices.size () == 1)
  {
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFilePLY (argv[ply_file_indices[0]], mesh);
    pcl::io::mesh2vtk (mesh, polydata1);
  }
  else if (obj_file_indices.size () == 1)
  {
    vtkSmartPointer<vtkOBJReader> readerQuery = vtkSmartPointer<vtkOBJReader>::New ();
    readerQuery->SetFileName (argv[obj_file_indices[0]]);
    readerQuery->Update ();
    polydata1 = readerQuery->GetOutput ();
  }

  //make sure that the polygons are triangles!
  vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New ();
#if VTK_MAJOR_VERSION < 6
  triangleFilter->SetInput (polydata1);
#else
  triangleFilter->SetInputData (polydata1);
#endif
  triangleFilter->Update ();

  vtkSmartPointer<vtkPolyDataMapper> triangleMapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
  triangleMapper->SetInputConnection (triangleFilter->GetOutputPort ());
  triangleMapper->Update();
  polydata1 = triangleMapper->GetInput();

  bool INTER_VIS = false;
  bool VIS = true;

  if (INTER_VIS)
  {
    visualization::PCLVisualizer vis;
    vis.addModelFromPolyData (polydata1, "mesh1", 0);
    vis.setRepresentationToSurfaceForAllActors ();
    vis.spin();
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointXYZ>);
  uniform_sampling (polydata1, SAMPLE_POINTS_, *cloud_1);

  if (INTER_VIS)
  {
    visualization::PCLVisualizer vis_sampled;
    vis_sampled.addPointCloud (cloud_1);
    vis_sampled.spin ();
  }

  // Voxelgrid
  VoxelGrid<PointXYZ> grid_;
  grid_.setInputCloud (cloud_1);
  grid_.setLeafSize (leaf_size, leaf_size, leaf_size);

  pcl::PointCloud<pcl::PointXYZ>::Ptr res(new pcl::PointCloud<pcl::PointXYZ>);
  grid_.filter (*res);

  pcl::PointCloud<pcl::PointXYZ>::Ptr res_move (new pcl::PointCloud<pcl::PointXYZ>);
  transform_dataset(res, res_move, division_, size_, name_number_);

  if (VIS)
  {
    visualization::PCLVisualizer vis3 ("VOXELIZED SAMPLES CLOUD");
    vis3.addPointCloud (res_move);
    vis3.spin ();
  }

  return (0);
}
