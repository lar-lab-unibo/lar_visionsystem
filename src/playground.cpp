#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

    
ros::Publisher pub;
//pcl::visualization::PCLVisualizer* viewer;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;
     
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new  pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::fromROSMsg(*input,*cloud);
    
  std::cout << "Received: "<<cloud->points.size()<<std::endl;
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;
    
 // viewer = new pcl::visualization::PCLVisualizer("viewer");
    
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, cloud_cb); 
  pub = nh.advertise<sensor_msgs::PointCloud2> ("mycloud", 1);
 
  // Spin
    while(nh.ok() ){
       // viewer->spinOnce();
        ros::spinOnce();
    }
}