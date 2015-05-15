#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <tf/transform_listener.h>
     #include "pcl_ros/transforms.h"
 #include "pcl_ros/impl/transforms.hpp"
#include <opencv2/opencv.hpp>

ros::Publisher pub;
//pcl::visualization::PCLVisualizer* viewer;

tf::Transform matToTF(cv::Mat& mat){
  tf::Vector3 origin;
  tf::Matrix3x3 tf3d;
  origin.setValue(
    static_cast<float>(mat.at<float>(0,3))/1000.0f,
    static_cast<float>(mat.at<float>(1,3))/1000.0f,
    static_cast<float>(mat.at<float>(2,3))/1000.0f
  );

  tf3d.setValue(
    static_cast<float>(mat.at<float>(0,0)), static_cast<float>(mat.at<float>(0,1)), static_cast<float>(mat.at<float>(0,2)),
    static_cast<float>(mat.at<float>(1,0)), static_cast<float>(mat.at<float>(1,1)), static_cast<float>(mat.at<float>(1,2)),
    static_cast<float>(mat.at<float>(2,0)), static_cast<float>(mat.at<float>(2,1)), static_cast<float>(mat.at<float>(2,2))
  );

  for(int i = 0; i < 3; i++){
    for(int j = 0; j < 3; j++){
      tf3d[i][j] = mat.at<float>(i,j);

    }
    std::cout<<std::endl;

  }


  tf::Quaternion tfqt;
  tf3d.getRotation(tfqt);
  tfqt = tfqt.normalized();

  tf::Transform transform;
  transform.setOrigin(origin);
  transform.setRotation(tfqt);
  return transform;
}

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
 /* sensor_msgs::PointCloud2 output;
     
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new  pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(*input, pcl_pc);
  pcl::fromPCLPointCloud2(pcl_pc, *cloud);
  

      
      sensor_msgs::PointCloud out;
        
    float dummy_query_data[10] = { 
        0,0,-1,0,
        0,1,0,0,
        1,0,0,0,
        0,0,0,1};
    
    cv::Mat dummy_query = cv::Mat(2, 4, CV_32F, dummy_query_data);
     tf::Transform tf = matToTF(dummy_query);
        tf::TransformListener listener;
    
    pcl_ros::transformPointCloud ("lar_marker_111", *input, out,listener);
      //tf::TransformListener::("lar_marker_111",*input,out);
    */
     sensor_msgs::PointCloud2 out = *input;
      out.header.frame_id = "camera_link";
     pub.publish(out);
    //viewer->addPointCloud(cloud, "scene");

  //std::cout << "Received: "<<cloud->points.size()<<std::endl;
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;
    
  //viewer = new pcl::visualization::PCLVisualizer("viewer");
    
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb); 
  pub = nh.advertise<sensor_msgs::PointCloud2> ("mycloud", 1);
 
  // Spin
    while(nh.ok()  ){
       
        ros::spinOnce();
    }
}