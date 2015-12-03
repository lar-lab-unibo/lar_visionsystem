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
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>

//CUSTOM NODES

#include "lar_tools.h"
#include "lar_vision/commons/lar_vision_commons.h"
#include "lar_vision/commons/Noiser.h"
#include "tsdf/tsdf_volume_octree.h"
#include "tsdf/marching_cubes_tsdf_octree.h"
#include "segmentation/HighMap.h"

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
pcl::PointCloud<PointType>::Ptr cloud_corrected(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_trans(new pcl::PointCloud<PointType>);
pcl::PointCloud<NormalType>::Ptr cloud_trans_normals(new pcl::PointCloud<NormalType>);
pcl::PointCloud<PointType>::Ptr cloud_trans_purged(new pcl::PointCloud<PointType>);

//VIEWER
bool show_normals = false;

//SEGMENTATION
pcl::PointCloud<PointType>::Ptr planes(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr clusters(new pcl::PointCloud<PointType>());
double segmentation_plane_slice = 0.01f;
double segmentation_plane_startz = -0.0f;
double segmentation_plane_height= 2.00;
int segmentation_plane_min_inliers= 50;
double segmentation_plane_angle_th= 10.0;
int segmentation_plane_speedup = 1;

double highest_plane_z = -100.0f;

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
int crop_width = width_;
int crop_height = height_;
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

/** ROBOT */
double robot_velocity = 0.0;
double robot_velocity_th = 0.00005;
int robot_standing_frames = 0;

/** INTEGRATING OPTIONS */
int slots = 30;
int data_to_consume = 0;
int consumed_data = 0;
double offx = 0;
double offy = 0;
double offz = 0;
int auto_integrate_stading_frames_th = 30;
bool auto_integrate_if_robot_is_standing = true;
std::string out_dir;
std::string output_name="live";

/** NODES */
ros::Subscriber sub_cloud;
ros::Subscriber sub_pose;
ros::Subscriber sub_joints_state;
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
 * Saves current TSDF to disk
 */
void save_current_tsdf(){
        MarchingCubesTSDFOctree mc;
        mc.setMinWeight (min_weight);
        mc.setInputTSDF (tsdf);
        mc.setColorByRGB (true);

        pcl::PolygonMesh::Ptr mesh (new pcl::PolygonMesh);
        mc.reconstruct (*mesh);
        pcl::PointCloud<PointType>::Ptr cloud_out(new pcl::PointCloud<PointType>);
        pcl::fromPCLPointCloud2(mesh->cloud, *cloud_out);
        pcl::transformPointCloud(*cloud_out,*cloud_out,first_T_0_CAMERA);

        std::string pcd_out_filename = out_dir + "/" +output_name+"_tsdf.pcd";
        std::string pcd_simple_out_filename = out_dir + "/" +output_name+"_tsdf_simple.pcd";
        std::string ply_out_filename = out_dir + "/" +output_name+"_tsdf_mesh.ply";

        ROS_INFO("Writing PCD: %s",pcd_out_filename.c_str());
        ROS_INFO("Writing PCD Simple: %s",pcd_simple_out_filename.c_str());
        ROS_INFO("Writing PLY: %s",ply_out_filename.c_str());

        pcl::io::savePCDFileBinary(pcd_out_filename, *cloud_out);
        pcl::io::savePCDFileBinary(pcd_simple_out_filename, *tsdf_cloud);
        pcl::io::savePLYFileBinary(ply_out_filename, *mesh);

        if(false) {
                std::string tsdf_out_filename = out_dir + "/" +output_name+"_tsdf.tsdf";
                ROS_INFO("Writing TSDF: %s",tsdf_out_filename.c_str());
                tsdf->save(tsdf_out_filename);
        }

}

/**
 * Extract simple cloud representation from tsdf
 */
void simple_cloud_from_tsdf_update(){

        tsdf_nodes_out.clear();
        tsdf_cloud->points.clear();
        tsdf->nodeList(tsdf_nodes_out);
        //ROS_INFO("Tsdf nodes: %d",tsdf_nodes_out.size());
        float x,y,z;
        float d,w;
        float color;
        //double min_w = 10000000.0;
        //double max_w = -10000000.0;
        for(int i = 0; i < tsdf_nodes_out.size(); i++) {
                tsdf_nodes_out[i]->getCenter (x,y,z);
                tsdf_nodes_out[i]->getData(d,w);
                //min_w = w < min_w ? w : min_w;
                //max_w = w > max_w ? w : max_w;
                if(w<min_weight) continue;
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
        //ROS_INFO("Min weight: %f, Max weight: %f",min_w,max_w);
        Eigen::Matrix4d t = first_T_0_CAMERA;
        pcl::transformPointCloud(*tsdf_cloud, *tsdf_cloud,t );

/*
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
 */
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

        tsdf->integrateCloud (*cloud, pcl::PointCloud<pcl::Normal> (),tsdf_affine_transform);
        consumed_data++;
        ROS_INFO("Integrated cloud %d",consumed_data);
}


void plane_segmentation(){
        //Segmentation
        planes->points.clear();
        clusters->points.clear();

        std::vector<int> filtered_indices;
        std::vector<int> planes_indices;


        HighMap map(
          segmentation_plane_height,
          segmentation_plane_slice,
          -segmentation_plane_startz,
          segmentation_plane_speedup
        );
        map.planesCheck(
                cloud_trans,
                cloud_trans_normals,
                filtered_indices,
                planes_indices,
                segmentation_plane_angle_th,
                segmentation_plane_min_inliers
                );
        pcl::copyPointCloud(*cloud_trans, filtered_indices, *clusters);
        pcl::copyPointCloud(*cloud_trans, planes_indices, *planes);
        highest_plane_z = map.highest_plane_z;
}

/**
 * Cloud Received
 */
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
        if(cloud_consuming) return;
        pcl::PCLPointCloud2 pcl_pc;
        pcl_conversions::toPCL(*input, pcl_pc);
        pcl::fromPCLPointCloud2(pcl_pc, *cloud);
        pcl::transformPointCloud(*cloud, *cloud_trans, T_0_CAMERA);


        lar_vision::compute_normals(cloud_trans,cloud_trans_normals);
        plane_segmentation();

        if(data_to_consume>0) {
                integrate_current_cloud();
                data_to_consume--;
                if(data_to_consume<=0) {
                        if(consumed_data%10==0)
                                simple_cloud_from_tsdf_update();
                }
        }


        /** VISUALIZATION */
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        if(show_raw_data) {
                viewer->addPointCloud(cloud_trans, "view");

                

                if(show_normals){
                  viewer->addPointCloudNormals<PointType,NormalType>(cloud_trans,cloud_trans_normals);
                }
                lar_vision::display_cloud(*viewer, planes, 0,255,0, 1, "planes");

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
 * Robot Joints State received
 */
void jointStateReceived( const sensor_msgs::JointState& msg ){

        robot_velocity = 0.0;
        for(int i = 0; i < msg.velocity.size(); i++) {
                robot_velocity+=  msg.velocity[i]* msg.velocity[i];
        }
        robot_velocity= sqrt(robot_velocity);
        if(robot_velocity<=robot_velocity_th) {
                robot_standing_frames++;
                ROS_INFO("Robot is standing %d",robot_standing_frames);
        }else{
                robot_standing_frames=0;
        }

        if(auto_integrate_if_robot_is_standing) {
                if(robot_standing_frames>=auto_integrate_stading_frames_th) {
                        robot_standing_frames = 0;
                        data_to_consume++;
                }
        }

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
                save_current_tsdf();
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
        nh->param<std::string>("out_folder", out_dir,"/home/daniele/temp/");

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
        nh->param<int>("width", width_, 640);
        nh->param<int>("height", height_, 480);
        nh->param<double>("tsdf_size", tsdf_size, 1.0);
        nh->param<double>("cell_size", cell_size, 0.01);
        nh->param<double>("trunc_dist_pos", trunc_dist_pos, 0.03);
        nh->param<double>("trunc_dist_neg", trunc_dist_neg, 0.03);
        nh->param<double>("max_sensor_dist", max_sensor_dist, 3.0);
        nh->param<double>("min_sensor_dist", min_sensor_dist, 0.0);
        nh->param<double>("min_weight",min_weight,0);
        nh->param<int>("crop_width", crop_width, width_);
        nh->param<int>("crop_height", crop_height, height_);
        nh->param<double>("fx", focal_length_x_, 525.);
        nh->param<double>("fy", focal_length_y_, 525.);
        nh->param<double>("cx", principal_point_x_, 319.5);
        nh->param<double>("cy", principal_point_y_, 239.5);

        //Acquisition parameters
        nh->param<int>("slots", slots, 30);
        nh->param<double>("offx", offx, 0.0);
        nh->param<double>("offy", offy, 0.0);
        nh->param<double>("offz", offz, 0.0);
        nh->param<int>("standing_frames",auto_integrate_stading_frames_th,5);
        nh->param<bool>("auto_integrate",auto_integrate_if_robot_is_standing,false);

        //Segmentation parameters
        nh->param<double>("segmentation_plane_slice", segmentation_plane_slice, 0.01);
        nh->param<double>("segmentation_plane_startz", segmentation_plane_startz, 0.00);
        nh->param<double>("segmentation_plane_height", segmentation_plane_height, 2.00);
        nh->param<int>("segmentation_plane_min_inliers", segmentation_plane_min_inliers, 50);
        nh->param<double>("segmentation_plane_angle_th", segmentation_plane_angle_th, 10.0);
        nh->param<int>("segmentation_plane_speedup", segmentation_plane_speedup, 1);

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
        tsdf->setImageCropSize(crop_width,crop_height);
        Eigen::Affine3d tsdf_global_transform;
        tsdf_global_transform = T_WORLD_0;
        //tsdf->setGlobalTransform(tsdf_global_transform);
        tsdf->reset ();


        /** NOSES */
        sub_cloud = nh->subscribe("/camera/depth_registered/points", 1, cloud_cb);
        sub_pose = nh->subscribe("/lar_comau/comau_full_state_publisher", 1, pose_cb);
        sub_joints_state = nh->subscribe( "/lar_comau/comau_joint_states",1,jointStateReceived );

        /** THREADS */
        boost::thread collectorThread(consumeData);


        while (nh->ok() && !viewer->wasStopped()) {

                viewer->spinOnce();
                ros::spinOnce();
        }

        collectorThread.join();

}
