#include <stdio.h>
#include <string.h>
#include <stdlib.h>


//ROS

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <kdl/frames_io.hpp>
#include "geometry_msgs/Pose.h"
#include <laser_geometry/laser_geometry.h>

//OPENCV
//#include <opencv2/opencv.hpp>
//#include <cv_bridge/cv_bridge.h>
//#include <image_transport/image_transport.h>


//PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/transforms.h>
#include <pcl/surface/mls.h>
#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl_ros/impl/transforms.hpp>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/filters/passthrough.h>
//#include <pcl/filters/fast_bilateral.h>

//CUSTOM NODES

#include "lar_tools.h"
#include "lar_vision/commons/lar_vision_commons.h"
//#include "lar_vision/commons/Noiser.h"


//boost
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>

using namespace std;
using namespace lar_vision;

//ROS
ros::NodeHandle* nh;


//CLOUDS & VIEWER
pcl::visualization::PCLVisualizer* viewer;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_0(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<PointType>::Ptr cloud_0_full(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_0_rgb(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_0_rgb_filtered(new pcl::PointCloud<PointType>);
std::string save_folder;

bool cloud_consuming = false;
bool manual_merge = false;
bool bilateral = false;
double bilateral_sd = 3.0f;
double bilateral_sr = 0.1f;
double max_z= 1.0;
int slots = 20;

ros::Subscriber sub_laser;
ros::Subscriber sub_pose;
//image_transport::Subscriber sub_rgb;
//image_transport::Subscriber sub_depth;


sensor_msgs::JointState joint_msg;
bool show_filtered = false;

/** TRANSFORMS */
Eigen::Matrix4f T_0_BASE;
Eigen::Matrix4f T_BASE_ROBOT;
Eigen::Matrix4f T_ROBOT_LASER;
Eigen::Matrix4f T_0_LASER;
Eigen::Matrix4f T_0_ROBOT;

int data_to_consume = 0;


/* LASER */
laser_geometry::LaserProjection projector_;



void
pose_cb(const geometry_msgs::Pose& pose) {

        lar_tools::create_eigen_4x4(pose,T_BASE_ROBOT);


        T_0_ROBOT = T_0_BASE * T_BASE_ROBOT;
        T_0_LASER = T_0_ROBOT * T_ROBOT_LASER;


        //std::cout << T_0_LASER<<std::endl;
        //  std::cout << T_BASE_ROBOT<<"\n#########\n"<<T_0_LASER<<"\n\n\n\n"<<std::endl;
        //std::cout << pose <<std::endl;

}
int save_counter = 0;

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
                           void* viewer_void) {
        //    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
        if (event.getKeySym() == "v" && event.keyDown()) {
                if(!show_filtered) {

                        // Create a KD-Tree
                        pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
                        // Output has the PointNormal type in order to store the normals calculated by MLS
                        pcl::PointCloud<pcl::PointNormal> mls_points;
                        // Init object (second point type is for the normals, even if unused)
                        pcl::MovingLeastSquares<PointType, pcl::PointNormal> mls;

                        mls.setComputeNormals (true);

                        // Set parameters
                        mls.setInputCloud (cloud_0_full);
                        mls.setPolynomialFit (true);
                        mls.setSearchMethod (tree);
                        mls.setSearchRadius (0.03);
                        // Reconstruct
                        mls.process (mls_points);

                        cloud_0_rgb_filtered->points.clear();
                        for(int i = 0; i < mls_points.points.size(); i++) {
                                PointType p;
                                p.x = mls_points.points[i].x;
                                p.y = mls_points.points[i].y;
                                p.z = mls_points.points[i].z;
                                p.r = 255;
                                p.g = 255;
                                p.b = 255;
                                cloud_0_rgb_filtered->points.push_back(p);
                        }

                }
                  show_filtered=!show_filtered;
        }
}



void scan_cb(const sensor_msgs::LaserScan::ConstPtr& scan){
        sensor_msgs::PointCloud2 cloud_ros;
        projector_.projectLaser(*scan, cloud_ros);

        pcl::PCLPointCloud2 pcl_pc;
        pcl_conversions::toPCL(cloud_ros, pcl_pc);
        pcl::fromPCLPointCloud2(pcl_pc, *cloud);



        pcl::transformPointCloud(*cloud, *cloud_0, T_0_LASER);


        cloud_0_rgb->points.clear();
        double z;
        for(int i = 0; i < cloud_0->points.size(); i++) {
                PointType p;
                if(fabs(cloud_0->points[i].y)>0.4)continue;
                if(fabs(cloud_0->points[i].z)<-0.6)continue;
                p.x = cloud_0->points[i].x;
                p.y = cloud_0->points[i].y;
                p.z = cloud_0->points[i].z;
                z = (cloud_0->points[i].z+2.0)/0.5;
                while(z>1.0) z-=1.0;

                p.r = z*255;
                p.g = 255;
                p.b = 255;
                cloud_0_rgb->points.push_back(p);
        }


        (*cloud_0_full)+=(*cloud_0_rgb);





        viewer->removeAllPointClouds();

        if(show_filtered){
            viewer->addPointCloud(cloud_0_rgb_filtered, "view");
        }else{
            viewer->addPointCloud(cloud_0_full, "view");
        }


        display_cloud(*viewer,cloud_0_rgb,255,0,0,4,"laser_scan");


}

/** MAIN NODE **/
int
main(int argc, char** argv) {

        // Initialize ROS
        ros::init(argc, argv, "comau_manual_laser");
        ROS_INFO("comau_manual_laser node started...");
        nh = new ros::NodeHandle("~");

        nh->param<bool>("manual_merge", manual_merge, false);
        nh->param<bool>("bilateral", bilateral, false);
        nh->param<double>("sd", bilateral_sd, 3.0);
        nh->param<double>("sr", bilateral_sr, 0.1);

        nh->param<int>("slots", slots, 20);
        nh->param<double>("max_z", max_z, 1.0);
/*
        dynamic_reconfigure::Server<trimod_gripper::LwrManualPhotographerConfig> srv;
        dynamic_reconfigure::Server<trimod_gripper::LwrManualPhotographerConfig>::CallbackType f;
        f = boost::bind(&callback, _1, _2);
        srv.setCallback(f);
 */

        /** VIEWER */
        viewer = new pcl::visualization::PCLVisualizer("viewer");
        viewer->registerKeyboardCallback(keyboardEventOccurred, (void*) &viewer);

        /** TRANSFORMS */
        lar_tools::create_eigen_4x4(0, 0, 0, 0, 0, 0, T_0_BASE);
        lar_tools::create_eigen_4x4(0, 0, 0, M_PI, M_PI/2.0, 0,T_ROBOT_LASER);

        sub_laser = nh->subscribe<sensor_msgs::LaserScan> ("/scan", 1, scan_cb);

        //  sub_cloud = nh->subscribe("/camera/depth_registered/points", 1, cloud_cb);
        sub_pose = nh->subscribe("/lar_comau/comau_full_state_publisher", 1, pose_cb);

        //image_transport::ImageTransport it(*nh);
        //sub_rgb = it.subscribe("/xtion/xtion/rgb/image_raw", 1, rgb_cb);
        //sub_depth = it.subscribe("/xtion/xtion/depth/image_raw", 1, depth_cb);

        // Spin
        //boost::thread collectorThread(consumeData);

        // Spin
        while (nh->ok() && !viewer->wasStopped()) {

                viewer->spinOnce();
                ros::spinOnce();
        }

        //  collectorThread.join();

}
