#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/time.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

class SimpleOpenNIProcessor
{
public:
  void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
  {
    static unsigned count = 0;
    static double last = pcl::getTime ();
    if (++count == 30)
    {
      double now = pcl::getTime ();
      std::cout << "distance of center pixel :" << cloud->points [(cloud->width >> 1) * (cloud->height + 1)].z << " mm. Average framerate: " << double(count)/double(now - last) << " Hz" <<  std::endl;
      count = 0;
      last = now;
    }
    pcl::io::savePCDFileBinary ("scene_points.pcd", *cloud);
    std::cerr << "Saved " << cloud->size () << " data points to scene_points.pcd" << std::endl;
    pcl::visualization::PCLVisualizer viz;
    viz.addPointCloud(cloud, "points");
    viz.spin ();

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr points_seg (new pcl::PointCloud<pcl::PointXYZRGBA>);
    points_seg = seg_planar (cloud);
    pcl::io::savePCDFileBinary ("scene_seg.pcd", *points_seg);
    std::cerr << "segment successfully the scene point cloud to scene_seg.pcd" << std::endl;
    pcl::visualization::PCLVisualizer viewer;
    viewer.addPointCloud(points_seg, "points_seg");
    viewer.spin ();

  }
  
  void run ()
  {
    // create a new grabber for OpenNI devices
    pcl::Grabber* interface = new pcl::OpenNIGrabber();

    // make callback function from member function
    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
      boost::bind (&SimpleOpenNIProcessor::cloud_cb_, this, _1);

    // connect callback function for desired signal. In this case its a point cloud with color values
    boost::signals2::connection c = interface->registerCallback (f);

    // start receiving point clouds
    interface->start ();

    // wait until user quits program with Ctrl-C, but no busy-waiting -> sleep (1);
    //    while (true)
    boost::this_thread::sleep (boost::posix_time::seconds (100));

    // stop the grabber
    interface->stop ();
  }

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr seg_planar (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
  {

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr points_1 (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr points (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());      
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGBA> segmentation;
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setMaxIterations(100);
    segmentation.setDistanceThreshold(0.01);
    segmentation.setInputCloud(cloud);
    segmentation.segment(*inliers, *coefficients);
    float a,b,c,d;
    a = coefficients->values[0];
    b = coefficients->values[1];
    c = coefficients->values[2];
    d = coefficients->values[3];

    double  D;
    if (inliers->indices.size() > 0) {
      for (int i=0;i<cloud->size();i++) {			
        D=pointToPlaneDistance(cloud->points[i],a,b,c,d);			
        if ((D < 1.5)&&(D> 0.1)) {
          points_1->push_back(cloud->points[i]);
        }				
      }
    }
        
    else std::cout<<"no plane is found"<<std::endl;
//    pcl::io::savePCDFileASCII ("planar_seg_first.pcd", points_1);
//    std::cerr << "Saved " << points_1.points.size () << " data points to planar_seg_first.pcd ." << std::endl;

    pcl::visualization::PCLVisualizer viewer;
    viewer.addPointCloud(points_1, "points_1");
    viewer.spin ();

    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(points_1);
    seg.segment(*inliers, *coefficients);
    a = coefficients->values[0];
    b = coefficients->values[1];
    c = coefficients->values[2];
    d = coefficients->values[3];

    if (inliers->indices.size() > 0) {
      for (int i=0;i<points_1->size();i++) {			
        D=pointToPlaneDistance(points_1->points[i],a,b,c,d);			
        if ((D < 1.5)&&(D> 0.1)) {
          points->push_back(points_1->points[i]);
        }				
      }
    }

    else *points = *points_1;
    return points;
  }

};

int main ()
{
  SimpleOpenNIProcessor v;
  v.run ();
  return (0);
}
