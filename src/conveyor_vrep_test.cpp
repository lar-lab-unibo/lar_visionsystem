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
#include <pcl/filters/passthrough.h>
#include <pcl/features/shot_lrf.h>

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

typedef pcl::PointXYZ PT;

pcl::PointCloud<PT>::Ptr cloud_raw(new pcl::PointCloud<PT>);
pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_trans(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_trans_over(new pcl::PointCloud<PointType>);
//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_0(new pcl::PointCloud<pcl::PointXYZ>);
std::string save_folder;

double conveyor_height = 0.4;
double alpha = 0.01;
double noise_mag = 0.01;
double approach_distance = 0.05;
int target_steps = 80;

ros::Subscriber sub_cloud;
ros::Publisher robot_pub;


sensor_msgs::JointState joint_msg;
bool show_filtered = false;

/** TRANSFORMS */
Eigen::Matrix4d T_0_CAMERA;
Eigen::Matrix4d T_0;
Eigen::Matrix4d previous_target_t;
Eigen::Vector3d previous_target_position;
ros::Time previous_target_time;
double target_speed = 0;
double fixed_speed = 0;
bool target_visible = false;
double target_counter = target_steps;
int data_to_consume = 0;
bool force_unlock = false;


void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
                           void* viewer_void) {
        //    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
        if (event.getKeySym() == "v" && event.keyDown()) {

        }
}

void computeRF(pcl::PointCloud<PointType>::Ptr& cloud,Eigen::Matrix4d& t){
        if(cloud->points.size()<=0) return;
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud, centroid);
        pcl::PointCloud<PointType>::Ptr centroid_cloud(new pcl::PointCloud<PointType>());
        PointType p_centroid;
        p_centroid.x = centroid(0);
        p_centroid.y = centroid(1);
        p_centroid.z = centroid(2);
        centroid_cloud->points.push_back(p_centroid);

        pcl::PointCloud<pcl::ReferenceFrame>::Ptr rf_cloud(new pcl::PointCloud<pcl::ReferenceFrame>());


        pcl::SHOTLocalReferenceFrameEstimation<PointType, pcl::ReferenceFrame> est;
        est.setInputCloud(centroid_cloud);
        est.setSearchSurface(cloud);
        est.setRadiusSearch(20000.0f);
        est.compute(*rf_cloud);

        pcl::ReferenceFrame rf;
        rf = rf_cloud->points[0];

        t <<
        rf.x_axis[0], rf.y_axis[0], rf.z_axis[0], p_centroid.x,
        rf.x_axis[1], rf.y_axis[1], rf.z_axis[1], p_centroid.y,
        rf.x_axis[2], rf.y_axis[2], rf.z_axis[2], p_centroid.z,
        0, 0, 0, 1;

        if(t(0,0)<0) {
                Eigen::Matrix4d rotz;
                lar_tools::rotation_matrix_4x4_d('z',M_PI,rotz);
                t = t* rotz;
        }
        if(t(2,2)<0) {
                Eigen::Matrix4d rotx;
                lar_tools::rotation_matrix_4x4_d('x',M_PI,rotx);
                t = t* rotx;
        }
}

void computeCentroid(pcl::PointCloud<PointType>::Ptr& cloud,Eigen::Matrix4d& centroid){
        Eigen::Vector4f ct;
        pcl::compute3DCentroid (*cloud, ct);
        lar_tools::create_eigen_4x4_d(0,0,0,0,0,0, centroid);
        for(int i = 0; i< 3; i++)
                centroid(i,3) = ct[i];

}

void getPosition(Eigen::Matrix4d& t,Eigen::Vector3d& p){
        p[0] = t(0,3);
        p[1] = t(1,3);
        p[2] = t(2,3);
}

void toColorCloud(pcl::PointCloud<PT>::Ptr& cloud_in,pcl::PointCloud<PointType>::Ptr& cloud_out){
        cloud_out->points.clear();
        for(int i = 0; i < cloud_in->points.size(); i++) {
                PointType p;
                PT pt = cloud_in->points[i];
                p.x = pt.x;
                p.y = pt.y;
                p.z = pt.z+ noise_mag*(double)(rand()%100)/100.0;
                p.r = 255;
                p.g = 255;
                p.b = 255;
                cloud_out->points.push_back(p);
        }
}

void
cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
        //if(cloud_consuming) return;
        pcl::PCLPointCloud2 pcl_pc;
        pcl_conversions::toPCL(*input, pcl_pc);
        pcl::fromPCLPointCloud2(pcl_pc, *cloud_raw);
        toColorCloud(cloud_raw,cloud);

        pcl::transformPointCloud(*cloud, *cloud_trans, T_0_CAMERA);

        // Create the filtering object
        pcl::PassThrough<PointType> pass;
        pass.setInputCloud (cloud_trans);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (conveyor_height, 3.0);
        //pass.setFilterLimitsNegative (true);
        pass.filter (*cloud_trans_over);

        pass.setInputCloud (cloud_trans_over);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (-2, -1.2);
        //pass.setFilterLimitsNegative (true);
        pass.filter (*cloud_trans_over);

        //CENTORID TEST
        Eigen::Matrix4d centroid;
        computeRF(cloud_trans_over,centroid);

        centroid(2,3) = centroid(2,3)+approach_distance;

        Eigen::Vector3d target_position;
        getPosition(centroid,target_position);

        ros::Duration delta = ros::Time::now() - previous_target_time;
        double delta_p = (target_position-previous_target_position).norm();

        if(cloud_trans_over->points.size()>5 && !target_visible) {
                if(!force_unlock) {
                        target_visible = true;
                        target_speed = delta_p/delta.toSec();
                        target_counter = target_steps;
                }
        }

        if((cloud_trans_over->points.size()<5 && target_visible) || target_counter<=0) {
                if(!force_unlock) {
                        target_visible = false;
                        fixed_speed = target_speed;
                        if(target_counter<=0)
                                force_unlock = true;
                }
        }


        target_speed = (1.0-alpha)*target_speed + alpha*delta_p/delta.toSec();
        ROS_INFO("Delta time: %f, Delta p:%f\n",delta.toSec(),delta_p);
        ROS_INFO("V: %f\n",target_speed);
        ROS_INFO("Counter: %f\n",target_counter);


        if(target_visible)
                previous_target_t = centroid;

        geometry_msgs::Pose pose;
        geometry_msgs::PoseStamped posest;
        if(target_visible) {
                lar_tools::eigen_4x4_to_geometrypose_d(centroid,pose);
        }else{
                previous_target_t(1,3) = previous_target_t(1,3) +fixed_speed*delta.toSec();
                lar_tools::eigen_4x4_to_geometrypose_d(previous_target_t,pose);
        }
        posest.pose = pose;
        robot_pub.publish(posest);

        previous_target_position = target_position;
        previous_target_time = ros::Time::now();

        if(target_visible)
                target_counter--;

        viewer->removeAllShapes();
        viewer->removeAllPointClouds();
        //viewer->addPointCloud(cloud_trans, "view");
        display_cloud(*viewer,cloud_trans, 255,0,0, 1.0, "full");
        display_cloud(*viewer,cloud_trans_over, 0,255,0, 2.0, "over");

        draw_reference_frame(*viewer, T_0_CAMERA, 1.0,"T_0_CAMERA");
        draw_reference_frame(*viewer, T_0, 1.0,"T_WORLD");
        draw_reference_frame(*viewer, centroid, 0.2,"CENTROID");

}

/** MAIN NODE **/
int
main(int argc, char** argv) {

        // Initialize ROS
        ros::init(argc, argv, "conveyor_vrep_test");
        ROS_INFO("conveyor_vrep_test node started...");
        nh = new ros::NodeHandle("~");
        nh->param<double>("conveyor_height", conveyor_height, 0.4);
        nh->param<double>("alpha", alpha, 0.01);
        nh->param<double>("noise_mag", noise_mag, 0.01);
        nh->param<double>("approach_distance", approach_distance, 0.05);
        nh->param<int>("target_steps", target_steps, 80);



        /** VIEWER */
        viewer = new pcl::visualization::PCLVisualizer("viewer");
        viewer->registerKeyboardCallback(keyboardEventOccurred, (void*) &viewer);

        /** TRANSFORMS */
        lar_tools::create_eigen_4x4_d(-1.6, -1.5, 1.5, 0, -M_PI, 0, T_0_CAMERA);
        lar_tools::create_eigen_4x4_d(0,0,0,0,0,0, T_0);


        sub_cloud = nh->subscribe("/vrep/kinect_depth", 1, cloud_cb);
        robot_pub = nh->advertise<geometry_msgs::PoseStamped>("/vrep/rf", 1);

        // Spin
        while (nh->ok() && !viewer->wasStopped()) {

                viewer->spinOnce();
                ros::spinOnce();
        }

        //  collectorThread.join();

}
