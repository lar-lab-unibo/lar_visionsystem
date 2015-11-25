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
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>


//PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

//CUSTOM NODES

#include "lar_tools.h"
#include "lar_vision/commons/lar_vision_commons.h"
#include "lar_vision/commons/Noiser.h"


//boost
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>

using namespace std;
using namespace lar_vision;

//ROS
ros::NodeHandle* nh;


//CLOUDS & VIEWER
pcl::visualization::PCLVisualizer* viewer;

pcl::PointCloud<PointType>::Ptr cloud_cut(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_trans(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_noise(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_trans_filtered(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_full_filtered(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
std::string save_folder;
Noiser noiser;
bool cloud_consuming = false;
bool manual_merge = false;
double max_z= 1.0;
int slot = 20;

ros::Subscriber sub_cloud;
ros::Subscriber sub_pose;
image_transport::Subscriber sub_rgb;
image_transport::Subscriber sub_depth;


sensor_msgs::JointState joint_msg;


/** TRANSFORMS */
Eigen::Matrix4f T_0_BASE;
Eigen::Matrix4f T_BASE_ROBOT;
Eigen::Matrix4f T_ROBOT_CAMERA;
Eigen::Matrix4f T_0_CAMERA;
Eigen::Matrix4f T_0_ROBOT;

int data_to_consume = 0;



void
cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
        if(cloud_consuming) return;
        pcl::PCLPointCloud2 pcl_pc;
        pcl_conversions::toPCL(*input, pcl_pc);
        pcl::fromPCLPointCloud2(pcl_pc, *cloud);

        pcl::PassThrough<PointType> pass;
        pass.setInputCloud (cloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0.0, max_z);
        //pass.setFilterLimitsNegative (true);
        pass.filter (*cloud_cut);

        pcl::transformPointCloud(*cloud_cut, *cloud_trans, T_0_CAMERA);




        viewer->removeAllPointClouds();
        viewer->addPointCloud(cloud_trans, "view");
        if(manual_merge)
          viewer->addPointCloud(cloud_full_filtered, "scene");
}

/*
   cv::Mat current_rgb;
   cv::Mat current_depth;
   bool saving = false;
   bool rgb_ready = false;
   bool depth_ready = false;
 */

/*
   void
   rgb_cb(const sensor_msgs::ImageConstPtr& msg) {
        if (saving) return;
        try {
                rgb_ready = false;
                current_rgb = cv_bridge::toCvCopy(msg, "bgr8")->image;
                rgb_ready = true;
        } catch (cv_bridge::Exception& e) {
                ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
   }

   void
   depth_cb(const sensor_msgs::ImageConstPtr& msg) {
        if (saving) return;
        try {
                depth_ready = false;
                current_depth = cv_bridge::toCvCopy(msg, "32FC1")->image;
                depth_ready = true;
        } catch (cv_bridge::Exception& e) {
                ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
   }
 */

void
pose_cb(const geometry_msgs::Pose& pose) {

        lar_tools::create_eigen_4x4(pose,T_BASE_ROBOT);


        T_0_ROBOT = T_0_BASE * T_BASE_ROBOT;
        T_0_CAMERA = T_0_ROBOT * T_ROBOT_CAMERA;

        //  std::cout << T_BASE_ROBOT<<"\n#########\n"<<T_0_CAMERA<<"\n\n\n\n"<<std::endl;
        //std::cout << pose <<std::endl;

}
int save_counter = 0;

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
                           void* viewer_void) {
        //    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
        if (event.getKeySym() == "v" && event.keyDown()) {
                if (cloud->points.size() > 1000) {
                        //(*cloud_full) += (*cloud_trans);
                        std::cout << manual_merge<<std::endl;
                        if(manual_merge){
                            (*cloud_full_filtered)+=(*cloud_trans);
                        }else{
                            data_to_consume = slot;
                        }
                }
        }
}

void consumeData(){
        std::string save_folder = "/home/daniele/temp/temp_clouds";
        while(nh->ok()) {

                if(data_to_consume>0) {
                        cloud_consuming = true;
                        if(cloud->points.size()>0 ) {


                                std::ofstream myfile;
                                std::stringstream ss;

                                ss.str("");
                                ss << save_folder << "/" << save_counter << ".pcd";
                                pcl::io::savePCDFileBinary(ss.str().c_str(), *cloud_cut);

                                ss.str("");
                                ss << save_folder << "/" << save_counter << "_robot.txt";
                                myfile.open(ss.str().c_str());
                                myfile << T_0_ROBOT;
                                myfile.close();

                                ss.str("");
                                ss << save_folder << "/" << save_counter << "_ee.txt";
                                myfile.open(ss.str().c_str());
                                myfile << T_ROBOT_CAMERA;
                                myfile.close();

                                ss.str("");
                                ss << save_folder << "/" << save_counter << ".txt";
                                myfile.open(ss.str().c_str());
                                myfile << T_0_CAMERA;
                                myfile.close();




                                std::cout << "Saving shot: "<<save_counter<<std::endl;
                                save_counter++;

                                data_to_consume--;
                        }
                        cloud_consuming = false;
                }
                boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        }

}

/** MAIN NODE **/
int
main(int argc, char** argv) {

        // Initialize ROS
        ros::init(argc, argv, "comau_manual_photographer");
        ROS_INFO("comau_manual_photographer node started...");
        nh = new ros::NodeHandle("~");

        nh->param<bool>("manual_merge", manual_merge, false);
        nh->param<int>("slot", slot, 20);
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
        lar_tools::create_eigen_4x4(0, 0, 0, 0,0, 0, T_0_BASE);
        Eigen::Matrix4f correction;
        lar_tools::create_eigen_4x4(0, 0, 0, 0,0, M_PI, correction);
        lar_tools::create_eigen_4x4(
                //  0.065f+0.003f,-0.025f,-0.095f-0.07f,0, M_PI, M_PI/2.0f,
                //    T_ROBOT_CAMERA);

                0.061,
                -0.0094,
                -0.1488,
                179.0 * M_PI/180.0f,
                0,
                -89.5 * M_PI/180.0f,
                T_ROBOT_CAMERA);
        T_ROBOT_CAMERA=T_ROBOT_CAMERA*correction;
        //        std::stringstream ss;
        //        ss << "/home/daniele/temp/" << ros::Time::now();
        //        save_folder = ss.str();
        //
        //        T <<
        //        1, 0, 0, 0,
        //        0, 1, 0, 0,
        //        0, 0, 1, 0,
        //        0, 0, 0, 1;


        //        transformMatrixT(0, 0, 2.0f, 0, M_PI, 0, Robot_Base);
        //        rotationMatrixT(0, 0, -M_PI / 2, EE);
        //        transformMatrixT(0, 0, 1.0f, 0, 0, 0, Target);


        //        boost::filesystem::create_directory(save_folder);

        sub_cloud = nh->subscribe("/camera/depth_registered/points", 1, cloud_cb);
        sub_pose = nh->subscribe("/lar_comau/comau_full_state_publisher", 1, pose_cb);

        //image_transport::ImageTransport it(*nh);
        //sub_rgb = it.subscribe("/xtion/xtion/rgb/image_raw", 1, rgb_cb);
        //sub_depth = it.subscribe("/xtion/xtion/depth/image_raw", 1, depth_cb);

        // Spin
        boost::thread collectorThread(consumeData);

        // Spin
        while (nh->ok() && !viewer->wasStopped()) {

                viewer->spinOnce();
                ros::spinOnce();
        }

        collectorThread.join();

}
