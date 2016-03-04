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

//config
#include <dynamic_reconfigure/server.h>
#include <lar_visionsystem/ComauManualCameraAlignConfig.h>


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
#include <pcl/filters/fast_bilateral.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>

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
pcl::PolygonMesh comau_mesh;
std::string save_folder;
Noiser noiser;
bool registered_cloud = true;
bool cloud_consuming = false;
bool manual_merge = false;
bool bilateral = false;
double bilateral_sd = 3.0f;
double bilateral_sr = 0.1f;
double max_z= 1.0;
int slots = 20;
bool show_comau_model = true;

ros::Subscriber sub_cloud;
ros::Subscriber sub_pose;
image_transport::Subscriber sub_rgb;
image_transport::Subscriber sub_depth;


sensor_msgs::JointState joint_msg;


/** TRANSFORMS */
Eigen::Matrix4d T_0_BASE;
Eigen::Matrix4d T_BASE_ROBOT;
Eigen::Matrix4d T_ROBOT_CAMERA;
Eigen::Matrix4d T_0_CAMERA;
Eigen::Matrix4d T_CAMERA_CORRECTION;
Eigen::Matrix4d T_0_ROBOT;

int data_to_consume = 0;





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

        lar_tools::create_eigen_4x4_d(pose,T_BASE_ROBOT);

        Eigen::Matrix4d T_ROBOT_CAMERA_FINAL = T_ROBOT_CAMERA * T_CAMERA_CORRECTION;


        T_0_ROBOT = T_0_BASE * T_BASE_ROBOT;
        T_0_CAMERA = T_0_ROBOT * T_ROBOT_CAMERA_FINAL;
        //std::cout << T_0_CAMERA<<std::endl;
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
                        if(manual_merge) {
                                (*cloud_full_filtered)+=(*cloud_trans);
                        }else{
                                data_to_consume = slots;
                        }
                }
        }

        if (event.getKeySym() == "b" && event.keyDown()) {
                show_comau_model = !show_comau_model;
                ROS_INFO("Show Model Change");
        }
}

void consume_cloud(){
        std::string save_folder = "/home/daniele/temp/temp_clouds";
        if(data_to_consume>0) {
                cloud_consuming = true;
                if(cloud->points.size()>0 ) {


                        std::ofstream myfile;
                        std::stringstream ss;

                        ss.str("");
                        ss << save_folder << "/" << save_counter << ".pcd";
                        pcl::io::savePCDFileBinary(ss.str().c_str(), *cloud);

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
}

void consumeData(){

        while(nh->ok()) {

                consume_cloud();
                boost::this_thread::sleep(boost::posix_time::milliseconds(50));
        }

}

void
cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
        //if(cloud_consuming) return;
        pcl::PCLPointCloud2 pcl_pc;
        pcl_conversions::toPCL(*input, pcl_pc);
        pcl::fromPCLPointCloud2(pcl_pc, *cloud);

        //  Eigen::Matrix4d roty;
        //  lar_tools::rotation_matrix_4x4_d('z',0,roty);
        //    pcl::transformPointCloud(*cloud, *cloud_trans, roty);

        std::cout << "T_ROBOT_CAMERA \n"<<T_ROBOT_CAMERA<<std::endl;

        std::cout << "T_O_CAMERA \n"<<T_0_CAMERA<<std::endl;


        pcl::transformPointCloud(*cloud, *cloud_trans, T_0_CAMERA);


        consume_cloud();


        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        lar_vision::draw_reference_frame(*viewer,  T_0_CAMERA, 0.5f,"T_0_CAMERA");

        if(registered_cloud) {
                viewer->addPointCloud(cloud_trans, "view");
        }else{
                lar_vision::display_cloud(*viewer, cloud_trans, 0,255,0, 1, "view");
        }

        if(show_comau_model) {
                viewer->addPolygonMesh(comau_mesh,"comau_base");
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.4,0.4,0.4, "comau_base");

        }

        if(manual_merge)
                viewer->addPointCloud(cloud_full_filtered, "scene");
}

/** CFG CALLBACK */
void cfg_callback(lar_visionsystem::ComauManualCameraAlignConfig& config, uint32_t level) {
        ROS_INFO("Reconfigure Request: %f",config.x);

        lar_tools::create_eigen_4x4_d(
                config.x,
                config.y,
                config.z,
                config.roll*M_PI/180.0,
                config.pitch*M_PI/180.0,
                config.yaw*M_PI/180.0,
                T_CAMERA_CORRECTION);

}

/** MAIN NODE **/
int
main(int argc, char** argv) {

        // Initialize ROS
        ros::init(argc, argv, "comau_manual_camera_align");
        ROS_INFO("comau_manual_camera_align node started...");
        nh = new ros::NodeHandle("~");


/*
        dynamic_reconfigure::Server<trimod_gripper::LwrManualPhotographerConfig> srv;
        dynamic_reconfigure::Server<trimod_gripper::LwrManualPhotographerConfig>::CallbackType f;
        f = boost::bind(&callback, _1, _2);
        srv.setCallback(f);
 */

        /** VIEWER */
        viewer = new pcl::visualization::PCLVisualizer("viewer");
        viewer->registerKeyboardCallback(keyboardEventOccurred, (void*) &viewer);

        //COMAU BASE
        {
                pcl::io::loadPolygonFileSTL("/home/daniele/comau_base.stl", comau_mesh);

                Eigen::Matrix4d comau_base;
                lar_tools::create_eigen_4x4_d(0,0,-0.445,0,0,0,comau_base);

                pcl::PointCloud<pcl::PointXYZ> cloudp2;
                pcl::fromPCLPointCloud2(comau_mesh.cloud, cloudp2);
                pcl::transformPointCloud(cloudp2, cloudp2, comau_base);
                pcl::toPCLPointCloud2(cloudp2, comau_mesh.cloud);
        }


        /** TRANSFORMS */
        lar_tools::create_eigen_4x4_d(0, 0, 0, 0,0, 0, T_0_BASE);
        Eigen::Matrix4d correction;
        lar_tools::create_eigen_4x4_d(0, 0, 0, 0,0, 0, correction);
        lar_tools::create_eigen_4x4_d(0,0,0,0,0,0, T_ROBOT_CAMERA);

        //T_ROBOT_CAMERA=T_ROBOT_CAMERA*correction;

        lar_tools::create_eigen_4x4_d(0, 0, 0, 0,0, 0, T_CAMERA_CORRECTION);

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

        if(registered_cloud) {
                sub_cloud = nh->subscribe("/camera/depth_registered/points", 1, cloud_cb);
        }else{
                sub_cloud = nh->subscribe("/camera/depth/points", 1, cloud_cb);
        }
        sub_pose = nh->subscribe("/lar_comau/comau_full_state_publisher", 1, pose_cb);


        //Configurations
        dynamic_reconfigure::Server<lar_visionsystem::ComauManualCameraAlignConfig> srv;
        dynamic_reconfigure::Server<lar_visionsystem::ComauManualCameraAlignConfig>::CallbackType f;
        f = boost::bind(&cfg_callback, _1, _2);
        srv.setCallback(f);


        // Spin
        while (nh->ok() && !viewer->wasStopped()) {

                viewer->spinOnce();
                ros::spinOnce();
        }

        //  collectorThread.join();

}
