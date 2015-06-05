#include <ros/ros.h>
#include <fstream> 
#include <string> 
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
using namespace std;

ros::Publisher pub;
int now;
int x=0;
int cycle=0;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*input, *cloud);
  if(cycle==2){
	  x++;
	  string name = "/home/donaxi/catkin_ws/src/registration_pcl/src/cloud";
	  name.append(boost::to_string(x));
	  name.append(".pcd");
	  printf("%s",name.c_str());
	  scanf("%d",&now);
	  //ofstream outfile(name.c_str());
	  printf("created file");
	  pcl::io::savePCDFile(name, *cloud);
	  //pcl::io::savePCDFileASCII ("name.pcd", *cloud);
          cycle=0;

  }
  cycle++;
  //Conversion y salida
  sensor_msgs::PointCloud2 output_reg;
  pcl::toROSMsg (*cloud, output_reg);
  

  // Publish the data
  pub.publish (output_reg);
}

int
main (int argc, char** argv)
{

  // Initialize ROS
  ros::init (argc, argv, "registration_pcl");
  ros::NodeHandle nh;


  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output_reg", 1);
  // Spin
  ros::spin();
  
}
