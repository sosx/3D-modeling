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
  //Variables iniciales
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*input, *cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered3(new pcl::PointCloud<pcl::PointXYZ>); 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transform(new pcl::PointCloud<pcl::PointXYZ>); 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);


  //Filtro pasabandas a un metro en x
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);


  // Filtro Voxel
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud_filtered);
  sor.setLeafSize (0.003f, 0.003f, 0.003f);
  sor.filter (*cloud_filtered2);
  
  //Outliers
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> out;
  out.setInputCloud (cloud_filtered2);
  out.setMeanK (50);
  out.setStddevMulThresh (1.0);
  out.filter (*cloud_filtered3);

  //transform
  Eigen::Affine3f transform_1;
  transform_1 = Eigen::Affine3f::Identity();
  
  float theta = -(M_PI/9);

  transform_1.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitX()));

  pcl::transformPointCloud (*cloud_filtered3, *cloud_transform, transform_1);
  

  //segmentation
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud_transform);
  seg.segment (*inliers, *coefficients);

  //extraction
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  
  extract.setInputCloud (cloud_transform);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*cloud_p);
  



  
  //saving
  if(cycle==2){
	  x++;
	  string name = "/home/donaxi/catkin_ws/src/registration_pcl/src/cloud";
	  name.append(boost::to_string(x));
	  name.append(".pcd");
	  printf("%s",name.c_str());
	  scanf("%d",&now);
	  //ofstream outfile(name.c_str());
	  printf("created file");
	  pcl::io::savePCDFile(name, *cloud_p);
	  //pcl::io::savePCDFileASCII ("name.pcd", *cloud);
          cycle=0;

  }
  cycle++;
  //Conversion y salida
  sensor_msgs::PointCloud2 output_reg;
  pcl::toROSMsg (*cloud_p, output_reg);
  

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

