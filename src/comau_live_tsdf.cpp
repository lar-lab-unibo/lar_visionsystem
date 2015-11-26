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
#include <pcl/filters/fast_bilateral.h>
#include <pcl/registration/icp.h>

//CUSTOM NODES

#include "lar_tools.h"
#include "lar_vision/commons/lar_vision_commons.h"
#include "lar_vision/commons/Noiser.h"
#include "tsdf/tsdf_volume_octree.h"
#include "tsdf/marching_cubes_tsdf_octree.h"

//boost
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>

using namespace std;
using namespace lar_vision;

//ROS
ros::NodeHandle* nh;


//CLOUDS & VIEWER
pcl::visualization::PCLVisualizer* viewer;
pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_reduced(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_corrected(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_trans(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_trans_purged(new pcl::PointCloud<PointType>);

std::string save_folder;
Noiser noiser;
bool cloud_consuming = false;
bool manual_merge = false;
bool bilateral = false;
double bilateral_sd = 3.0f;
double bilateral_sr = 0.1f;
double max_z= 1.0;


//TSDF PARAMETERS
TSDFVolumeOctree::Ptr tsdf;

int width_ = 640;
int height_ = 480;
int w = 100;
int h = 100;
double focal_length_x_ = 525.;
double focal_length_y_ = 525.;
double principal_point_x_ = 319.5;
double principal_point_y_ = 239.5;
double tsdf_size = 2;
double tsdf_res;
double cell_size = 0.005;
int num_random_splits = 1;
double min_sensor_dist = 0.0;
double max_sensor_dist = 3.0;
double integrate_color = true;
double trunc_dist_pos = 0.01;
double trunc_dist_neg = 0.01;
double min_weight = 0;
bool cloud_noise = true;
bool camera_noise = true;
bool do_simple_merge = false;
double leaf = 0.01f;

double camera_error_x = 0;
double camera_error_y = 0;
double camera_error_z = 0;
double camera_error_roll = 0;
double camera_error_pitch = 0;
double camera_error_yaw = 0;

/** INTEGRATING OPTIONS */
int slots = 30;
int data_to_consume = 0;
int consumed_data = 0;
double offx = 0;
double offy = 0;
double offz = 0;

/** NODES */
ros::Subscriber sub_cloud;
ros::Subscriber sub_pose;
image_transport::Subscriber sub_rgb;
image_transport::Subscriber sub_depth;


/** TRANSFORMS */
Eigen::Matrix4d T_0_BASE;
Eigen::Matrix4d T_BASE_ROBOT;
Eigen::Matrix4d T_ROBOT_CAMERA;
Eigen::Matrix4d T_0_CAMERA;
Eigen::Matrix4d first_T_0_CAMERA;
Eigen::Matrix4d T_0_ROBOT;
Eigen::Matrix4d T_CAMERA_ROTATION;
Eigen::Matrix4d T_CAMERA_ERROR;
Eigen::Matrix4d T_CURRENTPOSE_REALPOSE;
Eigen::Matrix4d T_WORLD_0;
Eigen::Matrix4d tsdf_transform;
Eigen::Affine3d tsdf_affine_transform;

/** TSDF REPRESENTATION */
vector<RGBNode::Ptr> tsdf_nodes_out;
pcl::PointCloud<PointType>::Ptr tsdf_cloud(new pcl::PointCloud<PointType>());

/** VIEWER OPTIONS */
bool show_raw_data = false;


/**
 * Current camera pose correction
 */
void current_pose_correction(){
        if(tsdf_cloud->points.size()==0) {
                ROS_INFO("No current cloud!!");
        }else{
                std::vector<int> v;
                pcl::removeNaNFromPointCloud(*cloud_trans,*cloud_trans_purged,v);
                pcl::IterativeClosestPoint<PointType,PointType> icp;

                icp.setInputCloud(cloud_trans_purged);
                icp.setInputTarget(tsdf_cloud);

                icp.setMaxCorrespondenceDistance (0.05);
                // Set the maximum number of iterations (criterion 1)
                icp.setMaximumIterations (55000);
                // Set the transformation epsilon (criterion 2)
                icp.setTransformationEpsilon (1e-8);
                // Set the euclidean distance difference epsilon (criterion 3)
                icp.setEuclideanFitnessEpsilon (1);



                pcl::PointCloud<PointType> Final;
                icp.align(*cloud_corrected);
                Eigen::Matrix4f temp = icp.getFinalTransformation ();
                for(int i = 0; i<4; i++)
                  for(int j = 0; j<4; j++)
                      T_CURRENTPOSE_REALPOSE(i,j)=temp(i,j);

                std::cout << "has converged?" << icp.hasConverged() << " score:\n ";
                ROS_INFO("cloud in: %d ; cloud out: %d",(int)cloud_trans->points.size(),(int)cloud_trans_purged->points.size());
        }
}

/**
 * Extract simple cloud representation from tsdf
 */
void simple_cloud_from_tsdf_update(){
  /*
        tsdf_nodes_out.clear();
        tsdf_cloud->points.clear();
        tsdf->nodeList(tsdf_nodes_out);
        ROS_INFO("Tsdf nodes: %d",tsdf_nodes_out.size());
        float x,y,z;
        float d,w;
        float color;
        for(int i = 0; i < tsdf_nodes_out.size(); i++) {
                tsdf_nodes_out[i]->getCenter (x,y,z);
                tsdf_nodes_out[i]->getData(d,w);
                PointType pt;
                pt.x = x;
                pt.y = y;
                pt.z = z;
                pt.r = 0; pt.g = 0; pt.b = 0;
                if(d<=0.0f && d>=-0.3f) {
                        tsdf_nodes_out[i]->getRGB(
                                pt.r,pt.g,pt.b
                                );
                        tsdf_cloud->points.push_back(pt);
                }
        }
        Eigen::Matrix4d t = first_T_0_CAMERA;
        pcl::transformPointCloud(*tsdf_cloud, *tsdf_cloud,t );
      */

        MarchingCubesTSDFOctree mc;
        mc.setMinWeight (min_weight);
        mc.setInputTSDF (tsdf);
        mc.setColorByRGB (true);

        pcl::PolygonMesh::Ptr mesh (new pcl::PolygonMesh);
        mc.reconstruct (*mesh);
        pcl::PointCloud<PointType>::Ptr cloud_out(new pcl::PointCloud<PointType>);
        pcl::fromPCLPointCloud2(mesh->cloud, *tsdf_cloud);
        Eigen::Matrix4d t = first_T_0_CAMERA;
        pcl::transformPointCloud(*tsdf_cloud,*tsdf_cloud,t);

}

/**
 * Integrate current cloud
 */
void integrate_current_cloud(){

        if(consumed_data==0) {
                first_T_0_CAMERA = T_WORLD_0*T_0_CAMERA;
        }

        tsdf_transform =  first_T_0_CAMERA.inverse () * T_0_CAMERA;
        tsdf_affine_transform  = tsdf_transform;

        if (cloud->height != height_ || cloud->width != width_)
                PCL_ERROR ("Error: cloud %d has size %d x %d, but TSDF is initialized for %d x %d pointclouds\n", consumed_data, cloud->width, cloud->height, width_, height_);

        tsdf->integrateCloud (*cloud_reduced, pcl::PointCloud<pcl::Normal> (),tsdf_affine_transform);
        consumed_data++;
        ROS_INFO("Integrated cloud %d",consumed_data);
}

/**
 * Cloud Received
 */
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
        if(cloud_consuming) return;
        pcl::PCLPointCloud2 pcl_pc;
        pcl_conversions::toPCL(*input, pcl_pc);
        pcl::fromPCLPointCloud2(pcl_pc, *cloud);


        cloud_reduced->points.clear();

        int startx = (width_-h)/2.0f;
        int starty = (height_-h)/2.0f;

        for(int x = startx; x < startx+w; x++){
            for(int y = starty; y < starty+h; y++){
              int index = x + y*width_;
              cloud_reduced->points.push_back(cloud->points[index]);
          }
        }
        cloud = cloud_reduced;

        if(bilateral) {
                pcl::FastBilateralFilter<PointType> bif;
                bif.setSigmaS(bilateral_sd);
                bif.setSigmaR(bilateral_sr);
                bif.setInputCloud(cloud);
                bif.applyFilter(*cloud);
        }


        if(data_to_consume>0) {
                integrate_current_cloud();
                data_to_consume--;
                if(data_to_consume<=0) {
                        simple_cloud_from_tsdf_update();
                }
        }
        pcl::transformPointCloud(*cloud, *cloud_trans, T_0_CAMERA);


        /** VISUALIZATION */
        viewer->removeAllPointClouds();

        if(show_raw_data) {
                viewer->addPointCloud(cloud_trans, "view");
                //lar_vision::display_cloud(*viewer, cloud_corrected, 255,0,0, 5, "corrected");

        }

        viewer->addPointCloud(tsdf_cloud,"tsdf");
}


/**
 * Robot Pose Received
 */
void pose_cb(const geometry_msgs::Pose& pose) {

        lar_tools::create_eigen_4x4_d(pose,T_BASE_ROBOT);

        T_0_ROBOT = T_0_BASE * T_BASE_ROBOT;
        T_0_CAMERA = T_0_ROBOT * T_ROBOT_CAMERA;
        T_0_CAMERA = T_0_CAMERA * T_CURRENTPOSE_REALPOSE;
}


/**
 *  3D Viewer Events
 */
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
                           void* viewer_void) {
        //    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
        if (event.getKeySym() == "v" && event.keyDown()) {
                if (cloud->points.size() > 1000) {
                        data_to_consume=slots;
                }
        }
        if (event.getKeySym() == "b" && event.keyDown()) {
                show_raw_data=!show_raw_data;
        }
        if (event.getKeySym() == "a" && event.keyDown()) {
                current_pose_correction();
        }
        if (event.getKeySym() == "j" && event.keyDown()) {
                lar_tools::create_eigen_4x4_d(0,0,0, 0,0, 0, T_CURRENTPOSE_REALPOSE);
        }

}

/**
 * Consuming Thread
 */
void consumeData(){
        while(nh->ok() && !viewer->wasStopped()) {

                boost::this_thread::sleep(boost::posix_time::milliseconds(50));
        }

}

/** MAIN NODE **/
int
main(int argc, char** argv) {

        // Initialize ROS
        ros::init(argc, argv, "comau_live_tsdf");
        ROS_INFO("comau_manual_photographer node started...");
        nh = new ros::NodeHandle("~");

        /** PARAMETERS */
        //nh->param<std::string>("folder", path, "~/temp/temp_clouds/");
        //nh->param<std::string>("output_name", output_name,"merge_results");
        //nh->param<std::string>("out_folder", out_dir,"~/temp/");

        //Camera paratemetrs
        nh->param<bool>("simple_merge", do_simple_merge, false);
        nh->param<bool>("camera_noise", camera_noise, false);
        nh->param<double>("camera_error_x", camera_error_x, 0.0);
        nh->param<double>("camera_error_y", camera_error_y, 0.0);
        nh->param<double>("camera_error_z", camera_error_z, 0.0);
        nh->param<double>("camera_error_roll", camera_error_roll, 0.0);
        nh->param<double>("camera_error_pitch", camera_error_pitch, 0.0);
        nh->param<double>("camera_error_yaw", camera_error_yaw, 0.0);


        //TSDF Paremeters
        nh->param<int>("width", w, 640);
        nh->param<int>("height", h, 480);
        nh->param<double>("tsdf_size", tsdf_size, 1.0);
        nh->param<double>("cell_size", cell_size, 0.01);
        nh->param<double>("trunc_dist_pos", trunc_dist_pos, 0.03);
        nh->param<double>("trunc_dist_neg", trunc_dist_neg, 0.03);
        nh->param<double>("max_sensor_dist", max_sensor_dist, 3.0);
        nh->param<double>("min_sensor_dist", min_sensor_dist, 0.0);

        nh->param<double>("fx", focal_length_x_, 525.);
        nh->param<double>("fy", focal_length_y_, 525.);
        nh->param<double>("cx", principal_point_x_, 319.5);
        nh->param<double>("cy", principal_point_y_, 239.5);

        //Acquisition parameters
        nh->param<int>("slots", slots, 30);
        nh->param<double>("offx", offx, 0.0);
        nh->param<double>("offy", offy, 0.0);
        nh->param<double>("offz", offz, 0.0);


        //Compute TSDF Resolution
        int desired_res = tsdf_size / cell_size;
        int n = 1;
        while (desired_res > n) n *= 2;
        tsdf_res = n;

        /** VIEWER */
        viewer = new pcl::visualization::PCLVisualizer("viewer");
        viewer->registerKeyboardCallback(keyboardEventOccurred, (void*) &viewer);

        /** TRANSFORMS */
        lar_tools::create_eigen_4x4_d(0,0,0, 0,0, 0, T_0_BASE);
        lar_tools::create_eigen_4x4_d(0,0,0, 0,0, 0, T_CURRENTPOSE_REALPOSE);
        lar_tools::create_eigen_4x4_d(offx, offy, offz, 0,0, 0, T_WORLD_0);
        lar_tools::create_eigen_4x4_d(0.061,-0.0094,-0.1488,179.0 * M_PI/180.0f,0,-89.5 * M_PI/180.0f,T_ROBOT_CAMERA);
        lar_tools::create_eigen_4x4_d(0, 0, 0, 0,0, M_PI, T_CAMERA_ROTATION);   //Rotate camera for Geometry Construction

        T_ROBOT_CAMERA=T_ROBOT_CAMERA*T_CAMERA_ROTATION;

        //Camera noise correction
        if(camera_noise) {
                lar_tools::create_eigen_4x4_d(
                        (float)camera_error_x,(float)camera_error_y,(float)camera_error_z,
                        (float)camera_error_roll,(float)camera_error_pitch,(float)camera_error_yaw,
                        T_CAMERA_ERROR
                        );

                T_ROBOT_CAMERA=T_ROBOT_CAMERA*T_CAMERA_ERROR;
        }

        /** TSDF INITIALIZATION */
        tsdf.reset (new TSDFVolumeOctree);
        tsdf->setGridSize (tsdf_size, tsdf_size, tsdf_size);
        ROS_INFO("Setting resolution: %f with grid size %f and voxel size %f\n", tsdf_res, tsdf_size,cell_size);
        tsdf->setResolution (tsdf_res, tsdf_res, tsdf_res);
        tsdf->setImageSize (width_, height_);
        tsdf->setCameraIntrinsics (focal_length_x_, focal_length_y_, principal_point_x_, principal_point_y_);
        tsdf->setNumRandomSplts (num_random_splits);
        tsdf->setSensorDistanceBounds (min_sensor_dist, max_sensor_dist);
        tsdf->setIntegrateColor (integrate_color);
        tsdf->setDepthTruncationLimits (trunc_dist_pos, trunc_dist_neg);
        Eigen::Affine3d tsdf_global_transform;
        tsdf_global_transform = T_WORLD_0;
        //tsdf->setGlobalTransform(tsdf_global_transform);
        tsdf->reset ();


        /** NOSES */
        sub_cloud = nh->subscribe("/camera/depth_registered/points", 1, cloud_cb);
        sub_pose = nh->subscribe("/lar_comau/comau_full_state_publisher", 1, pose_cb);


        /** THREADS */
        boost::thread collectorThread(consumeData);


        while (nh->ok() && !viewer->wasStopped()) {

                viewer->spinOnce();
                ros::spinOnce();
        }

        collectorThread.join();

}
