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
#include <tf/transform_broadcaster.h>

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
#include "lar_tool_utils/UDPNode.h"
#include "lar_vision/commons/lar_vision_commons.h"
#include "lar_vision/commons/Noiser.h"
#include "tsdf/tsdf_volume_octree.h"
#include "tsdf/marching_cubes_tsdf_octree.h"
#include "segmentation/HighMap.h"
#include "segmentation/OnePointRansac.h"
#include "grasping/Slicer.h"
#include "grasping/grippers/CrabbyGripper.h"

//boost
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>

//COMMUNICATION DEFINES
#define COMMAND_SEND_JOINTS 333
#define COMMAND_SEND_CARTESIAN 777

using namespace std;
using namespace lar_vision;

//COMMUNICATION
struct MyUDPMessage {
        int command;
        long time;
        float payload[32];
};
lar_tools::UDPNode* robot_node;
MyUDPMessage send_message;

//ROS
ros::NodeHandle* nh;
tf::TransformBroadcaster* tf_broadcaster;


//CLOUDS & VIEWER
pcl::visualization::PCLVisualizer* viewer;
pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_corrected(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_trans(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_trans_bounded(new pcl::PointCloud<PointType>);
pcl::PointCloud<NormalType>::Ptr cloud_trans_normals(new pcl::PointCloud<NormalType>);
pcl::PointCloud<PointType>::Ptr cloud_trans_purged(new pcl::PointCloud<PointType>);
pcl::PolygonMesh comau_mesh;
pcl::PolygonMesh comau_mesh_original;
pcl::PolygonMesh gripper_mesh;
pcl::PolygonMesh gripper_mesh_ontarget;
pcl::PolygonMesh gripper_mesh_original;

//VIEWER
bool show_normals = false;
Palette common_palette;

//SEGMENTATION
struct TargetObject {
        pcl::PointCloud<PointType>::Ptr cloud;
        Eigen::Vector4f centroid;

        TargetObject(pcl::PointCloud<PointType>::Ptr& cloud_in){
                cloud = cloud_in;
                pcl::compute3DCentroid(*cloud, centroid);
        }

};

struct TargetObjectSorterX
{
        inline bool operator() (const TargetObject& obj1, const TargetObject& obj2)
        {
                return obj1.centroid[0] < obj2.centroid[0];
        }
};

pcl::PointCloud<PointType>::Ptr planes(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr clusters(new pcl::PointCloud<PointType>());
pcl::PassThrough<PointType> segmentation_plane_pass_filter;
bool segmentation_plane_success = false;
double segmentation_plane_highest_th = 0.03;
double segmentation_plane_slice = 0.01f;
double segmentation_plane_startz = -0.0f;
double segmentation_plane_height= 2.00;
int segmentation_plane_min_inliers= 50;
double segmentation_plane_angle_th= 10.0;
int segmentation_plane_speedup = 1;
double highest_plane_z = -100.0f;
std::vector<pcl::PointIndices> tsdf_clusters_indices;
std::vector<TargetObject> tsdf_clusters;
bool show_tsdf_clusters = false;
bool correcting_pose = false;
int cluster_consumed = -1;



//GRASPING
Grasper* grasper = NULL;
CrabbyGripper* gripper = NULL;
std::vector<int> grasp_indices;
bool grasping_do_grasp = false;
double grasp_object_slice_size = 0.01;
double grasp_object_slice_h = 0.05;
double grasp_hull_alpha = 0.1;
double grasp_hull_delta = 0.01;
double grasp_min_offset = 0.01;
double grasp_max_offset = 0.2;
double grasp_min_radius = 0.01;
double grasp_max_radius = 0.05;
double grasp_fritction_cone_angle = 1.57;
double grasp_max_curvature = 1.57;
double grasp_approach_angle_offset = 0.707;
int grasp_type = CRABBY_GRIPPER_STATUS_DUAL;
bool target_object_acquired = false;
pcl::PointCloud<PointType>::Ptr target_object(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr target_object_approach_slice(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr target_grasp_configuration_cloud(new pcl::PointCloud<PointType>());
Eigen::Matrix4d target_object_rf;
Eigen::Matrix4d target_object_approach_rf;
Eigen::Matrix4d target_object_approach_rf_wrist;
Eigen::Matrix4d target_object_approach_rf_wrist_waypoint;
bool target_object_approach_rf_wrist_ready = false;
bool show_target_object_slices = false;
std::vector<pcl::PointIndices> target_object_slice_indices;
double apporach_offset_distance = 0.30;

//TEST
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
double bound_x_min = 0.0f;
double bound_x_max = 1.5;
double bound_y_min = -0.4;
double bound_y_max = 0.4;

double leaf = 0.01f;
bool use_marching_cube = true;

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

                icp.setInputSource(cloud_trans_purged);
                icp.setInputTarget(tsdf_cloud);

                // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
                icp.setMaxCorrespondenceDistance (0.05);
                // Set the maximum number of iterations (criterion 1)
                icp.setMaximumIterations (50);
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


        if(do_simple_merge) {

                cloud_trans_bounded->points.clear();

                pcl::PassThrough<PointType> bound_filter;
                bound_filter.setFilterFieldName ("x");
                bound_filter.setFilterLimits (bound_x_min, bound_x_max);
                bound_filter.setInputCloud (cloud_trans);
                bound_filter.filter (*cloud_trans_bounded);

                bound_filter.setFilterFieldName ("y");
                bound_filter.setFilterLimits (bound_y_min, bound_y_max);
                bound_filter.setInputCloud (cloud_trans_bounded);
                bound_filter.filter (*cloud_trans_bounded);

                (*tsdf_cloud) += (*cloud_trans_bounded);
                return;
        }

        if(use_marching_cube) {

                MarchingCubesTSDFOctree mc;
                mc.setMinWeight (min_weight);
                mc.setInputTSDF (tsdf);
                mc.setColorByRGB (true);

                pcl::PolygonMesh::Ptr mesh (new pcl::PolygonMesh);
                mc.reconstruct (*mesh);
                pcl::PointCloud<PointType>::Ptr cloud_out(new pcl::PointCloud<PointType>);
                pcl::fromPCLPointCloud2(mesh->cloud, *tsdf_cloud);
                pcl::transformPointCloud(*tsdf_cloud,*tsdf_cloud,first_T_0_CAMERA);
        }else{
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
                        if(d<=0.1f) {
                                tsdf_nodes_out[i]->getRGB(
                                        pt.r,pt.g,pt.b
                                        );
                                tsdf_cloud->points.push_back(pt);
                        }
                }
                Eigen::Matrix4d t = first_T_0_CAMERA;
                pcl::transformPointCloud(*tsdf_cloud, *tsdf_cloud,t );
        }



        //Highets plane segmentation
        if(segmentation_plane_success) {
                segmentation_plane_pass_filter.setInputCloud (tsdf_cloud);
                segmentation_plane_pass_filter.filter (*tsdf_cloud);
        }

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


        bool save_per_frame = false;
        if(save_per_frame) {
                std::ofstream myfile;
                std::stringstream ss;
                ss << "/home/daniele/temp/live/"<<consumed_data<<".pcd";
                pcl::io::savePCDFileBinary(ss.str(), *cloud);

                ss.str("");
                ss << "/home/daniele/temp/live/"<<consumed_data<<"_robot.txt";
                myfile.open(ss.str().c_str());
                myfile << T_0_ROBOT;
                myfile.close();

                ss.str("");
                ss <<  "/home/daniele/temp/live/" << consumed_data << "_ee.txt";
                myfile.open(ss.str().c_str());
                myfile << T_ROBOT_CAMERA;
                myfile.close();

                ss.str("");
                ss <<  "/home/daniele/temp/live/" << consumed_data << ".txt";
                myfile.open(ss.str().c_str());
                myfile << T_0_CAMERA;
                myfile.close();

        }


        tsdf->integrateCloud (*cloud, pcl::PointCloud<pcl::Normal> (),tsdf_affine_transform);
        consumed_data++;
        ROS_INFO("Integrated cloud %d",consumed_data);
}


void adjust_gripper_model_transform(  Eigen::Matrix4d& t){
        Eigen::Matrix4d roty,rotz;
        lar_tools::rotation_matrix_4x4_d('y',-M_PI/2.0,roty);
        lar_tools::rotation_matrix_4x4_d('z',M_PI,rotz);
        t = t * rotz;
        t = t * roty;
        t = t * rotz;
}

/**
 * Rotate Gripper Mesh
 */
void transform_gripper_ontarget_mesh(){
        Eigen::Matrix4d gripper_ontarget_tf = target_object_approach_rf;
        adjust_gripper_model_transform(gripper_ontarget_tf);

        pcl::PointCloud<pcl::PointXYZ> cloudp;
        pcl::fromPCLPointCloud2(gripper_mesh_original.cloud, cloudp);
        pcl::transformPointCloud(cloudp, cloudp, gripper_ontarget_tf);
        pcl::toPCLPointCloud2(cloudp, gripper_mesh_ontarget.cloud);
}

/**
 * Rotate Gripper Mesh
 */
void transform_gripper_mesh(){
        Eigen::Matrix4d wrist_offset;
        lar_tools::create_eigen_4x4_d(-0.025,0.0,-0.07-0.095,0,0,0,wrist_offset);
        Eigen::Matrix4d gripper_tf = T_0_ROBOT*wrist_offset;
        adjust_gripper_model_transform(gripper_tf);

        pcl::PointCloud<pcl::PointXYZ> cloudp2;
        pcl::fromPCLPointCloud2(gripper_mesh_original.cloud, cloudp2);
        pcl::transformPointCloud(cloudp2, cloudp2, gripper_tf);
        pcl::toPCLPointCloud2(cloudp2, gripper_mesh.cloud);
}

/**
 * Approach RF Refinement by means of CRASP TYPE
 */
void compute_approach_rf_refinement(){

        if(gripper==NULL) return;

        double x,y,z,roll,pitch,yaw;
        gripper->getApproachRF(grasper->points, grasp_indices,x,y,z,roll,pitch,yaw);
        z = target_object_approach_slice->points[0].z;
        target_object_approach_rf = Eigen::Matrix4d::Identity();
        target_object_approach_rf_wrist = Eigen::Matrix4d::Identity();
        target_object_approach_rf_wrist_waypoint = Eigen::Matrix4d::Identity();
        lar_tools::create_eigen_4x4_d(x,y,z,roll,pitch,yaw,target_object_approach_rf);

        if(grasp_type==CRABBY_GRIPPER_STATUS_PARALLEL) {
                Eigen::Matrix4d roty,rotz,wrist_offset;
                lar_tools::rotation_matrix_4x4_d('y',-M_PI/2.0,roty);
                lar_tools::rotation_matrix_4x4_d('z',M_PI/2.0,rotz);
                lar_tools::create_eigen_4x4_d(0.025,0.0,0.07+0.095,0,0,0,wrist_offset);
                target_object_approach_rf = target_object_approach_rf*roty;

                target_object_approach_rf_wrist = target_object_approach_rf*wrist_offset;

                Eigen::Matrix4d approach_offset;
                lar_tools::create_eigen_4x4_d(apporach_offset_distance,0,apporach_offset_distance,0,0,0,approach_offset);
                target_object_approach_rf_wrist_waypoint = target_object_approach_rf_wrist*approach_offset;

                /*
                   std::cout << "WRIST"<<std::endl;
                   std::cout << target_object_approach_rf_wrist<<std::endl;
                   std::cout << "OFFSET"<<std::endl;
                   std::cout << approach_offset<<std::endl;
                   std::cout << "WRIST WAY"<<std::endl;
                   std::cout << target_object_approach_rf_wrist_waypoint<<std::endl;
                 */

        }else if(grasp_type==CRABBY_GRIPPER_STATUS_DUAL) {

                Eigen::Matrix4d roty,rotz,wrist_offset;
                lar_tools::rotation_matrix_4x4_d('z',M_PI/2.0,rotz);
                lar_tools::create_eigen_4x4_d(0.025,0.0,0.07+0.095,0,0,0,wrist_offset);
                target_object_approach_rf = target_object_approach_rf*rotz;
                target_object_approach_rf_wrist = target_object_approach_rf*wrist_offset;
                Eigen::Matrix4d approach_offset;
                lar_tools::create_eigen_4x4_d(0,0,apporach_offset_distance,0,0,0,approach_offset);
                target_object_approach_rf_wrist_waypoint = target_object_approach_rf_wrist*approach_offset;

        }


}

/**
 * Check Approach RF Consistency
 */
bool check_approach_rf_consistency(){
        if(grasp_type==CRABBY_GRIPPER_STATUS_PARALLEL) {
                Eigen::Vector3f base_x;
                base_x<< 1,0,0;
                Eigen::Vector3f approach_dir;
                approach_dir<<
                -target_object_approach_rf(0,2),
                -target_object_approach_rf(1,2),
                -target_object_approach_rf(2,2);

                double approach_angle_offset = acos(approach_dir.dot(base_x));
                return fabs(approach_angle_offset)<=grasp_approach_angle_offset;
        }else if(grasp_type==CRABBY_GRIPPER_STATUS_DUAL) {
                return true;
        }
}

/**
 * //TODO: INCAPSULARE!!!!! CHE è STO SCHIFO??
 */
void compute_target_object_approach(){
        ROS_INFO("Computing Target Approach: %d",(int)(target_object_approach_slice->points.size()));
        if(target_object_approach_slice->points.size()<=0 || !target_object_acquired) return;

        grasper = new Grasper(grasp_hull_alpha, grasp_hull_delta);
        grasper->hull_type = LAR_VISION_GRASPER_HULL_TYPE_CONCAVE;
        grasper->setCloud(target_object_approach_slice);
        display_cloud(*viewer, grasper->hull, 255, 0, 0, 15, "target_object_approach_hull");

        gripper = new CrabbyGripper(grasp_type);
        gripper->auto_discard_planar_invalids = true;
        gripper->min_offset = grasp_min_offset;
        gripper->max_offset = grasp_max_offset;
        gripper->min_radius = grasp_min_radius;
        gripper->max_radius = grasp_max_radius;
        gripper->fritction_cone_angle = grasp_fritction_cone_angle;
        gripper->max_curvature = grasp_max_curvature;

        grasp_indices.clear();
        int jumps = 0;
        bool found = false;
        target_object_approach_rf_wrist_ready = false;
        Eigen::Matrix4d local_transform= Eigen::Matrix4d::Identity();

        while(!found) {
                gripper->find(grasper->points, grasp_indices,jumps);
                if(grasp_indices.size()==0) break;

                ROS_INFO("Grasp FOUND!");





                compute_approach_rf_refinement();


                if(!check_approach_rf_consistency()) {
                        jumps++;
                        continue;
                }else{
                        found = true;
                        target_object_approach_rf_wrist_ready = true;
                        double sx,sy,sz,sroll,spitch,syaw;
                        lar_tools::eigen_4x4_to_xyzrpy_d(target_object_approach_rf_wrist,sx,sy,sz,sroll,spitch,syaw);
                        ROS_INFO("APPROACH WRIST POSITION %f,%f,%f    ,%f,%f,%f",sx,sy,sz,sroll,spitch,syaw);

                        //TFs
                        tf::Transform target_object_approach_rf_wrist_tf;
                        tf::Transform target_object_approach_offset_rf_wrist_tf;
                        lar_tools::eigen_4x4_d_to_tf(target_object_approach_rf_wrist,target_object_approach_rf_wrist_tf);
                        lar_tools::eigen_4x4_d_to_tf(target_object_approach_rf_wrist_waypoint,target_object_approach_offset_rf_wrist_tf);

                        tf_broadcaster->sendTransform(tf::StampedTransform(target_object_approach_rf_wrist_tf, ros::Time::now(), "base", "approach_position"));
                        tf_broadcaster->sendTransform(tf::StampedTransform(target_object_approach_offset_rf_wrist_tf, ros::Time::now(), "base", "approach_position_waypoint"));
                        transform_gripper_ontarget_mesh();
                }
                if (grasp_indices.size() > 0) {
                        target_grasp_configuration_cloud->points.clear();

                        for (int i = 0; i < grasp_indices.size(); i++) {
                                PointType p;
                                p.x = grasper->points[grasp_indices[i]].p[0];
                                p.y = grasper->points[grasp_indices[i]].p[1];
                                p.z = target_object_approach_slice->points[0].z;
                                target_grasp_configuration_cloud->points.push_back(p);
                        }
                        break;
                }
        }
}

/**
 * Sends Over UDP Target Approach Waypoint
 */
void send_target_approach_to_robot(){
        if(target_object_approach_rf_wrist_ready) {
                double sx,sy,sz,sroll,spitch,syaw;
                lar_tools::eigen_4x4_to_xyzrpy_d(target_object_approach_rf_wrist_waypoint,sx,sy,sz,sroll,spitch,syaw);
                ROS_INFO("APPROACH WAYPOINT WRIST POSITION %f,%f,%f    ,%f,%f,%f",sx,sy,sz,sroll,spitch,syaw);

                send_message.command = COMMAND_SEND_CARTESIAN;
                send_message.time = -1;
                send_message.payload[0] = sx*1000.0;
                send_message.payload[1] = sy*1000.0;
                send_message.payload[2] = sz*1000.0;
                send_message.payload[3] = sroll*180.0/M_PI;
                send_message.payload[4] = spitch*180.0/M_PI;
                send_message.payload[5] = syaw*180.0/M_PI;
                robot_node->send((void *)&send_message,sizeof(send_message));
        }
}

/**
 * Slices Target Object in Tsdf
 */
void slice_target_object(){
        if(target_object->points.size()>0 && target_object_acquired) {
                ROS_INFO("Computing Object Slices");
                Slicer slicer(grasp_object_slice_size);
                Eigen::Vector4f centroid;
                pcl::compute3DCentroid(*target_object, centroid);

                target_object_rf = Eigen::Matrix4d::Identity();
                for(int i = 0; i < 3; i++)
                        target_object_rf(i,3) = centroid[i];


                slicer.slice(target_object, target_object_slice_indices);

                int target_index = 0;
                if(grasp_object_slice_h>=0) {
                        target_index = target_object_slice_indices.size()>grasp_object_slice_h ? target_object_slice_indices.size() - grasp_object_slice_h : target_object_slice_indices.size()/2;
                }else{
                        target_index = target_object_slice_indices.size()>fabs(grasp_object_slice_h) ? -grasp_object_slice_h : target_object_slice_indices.size()/2;
                }
                ROS_INFO("Computed Object Slices! Slice %d of %d",target_index,(int)target_object_slice_indices.size());
                pcl::copyPointCloud(*target_object, target_object_slice_indices[target_index].indices, *target_object_approach_slice);

        }

}



/**
 * Clusterizes Common TSDF
 */
void clusterize_tsdf(){

        tsdf_clusters_indices.clear();
        tsdf_clusters.clear();

        clusterize(tsdf_cloud, tsdf_clusters_indices);

        if(tsdf_clusters_indices.size()>0) {
                for(int i = 0; i < tsdf_clusters_indices.size(); i++) {
                        if(tsdf_clusters_indices[i].indices.size()>50) {
                                pcl::PointCloud<PointType>::Ptr cluster(new pcl::PointCloud<PointType>());
                                pcl::copyPointCloud(*tsdf_cloud, tsdf_clusters_indices[i].indices, *cluster);
                                tsdf_clusters.push_back(TargetObject(cluster));
                        }
                }
        }
        std::sort(tsdf_clusters.begin(), tsdf_clusters.end(), TargetObjectSorterX());
        target_object_acquired = false;
        if(tsdf_clusters.size()>0) {
                if(cluster_consumed>=0 && (cluster_consumed+1)<tsdf_clusters.size()) {
                        target_object = tsdf_clusters[cluster_consumed+1].cloud;
                        target_object_acquired=true;
                }
                else{
                        target_object = tsdf_clusters[0].cloud;
                        target_object_acquired=true;
                }
        }
        ROS_INFO("Tsdf Clusters: %d",(int)tsdf_clusters.size());
}

/**
 * Planes Segmentation
 */
void plane_segmentation(){
        //Segmentation
        planes->points.clear();
        clusters->points.clear();

        std::vector<int> filtered_indices;
        std::vector<int> planes_indices;



/*
        OnePointRansac one(segmentation_plane_slice,0.99,1000,std::cos(segmentation_plane_angle_th*M_PI/180.0));

        one.planesCheck(
                cloud_seg,
                cloud_seg_normals,
                filtered_indices,
                planes_indices,
                segmentation_plane_angle_th,
                segmentation_plane_min_inliers
                );
 */

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

        //ROS_INFO("Segmentation MIN Z: %f",map.min_z);
        //ROS_INFO("Segmentation MAX Z: %f",map.max_z);

        pcl::copyPointCloud(*cloud_trans, filtered_indices, *clusters);
        pcl::copyPointCloud(*cloud_trans, planes_indices, *planes);

        highest_plane_z = map.highest_plane_z + segmentation_plane_highest_th;
        //ROS_INFO("Segmentation Highest Z: %f",highest_plane_z);
        segmentation_plane_pass_filter.setFilterFieldName ("z");
        segmentation_plane_pass_filter.setFilterLimits (highest_plane_z, 3000.0);
        segmentation_plane_success = planes->points.size()>10;

}




/**
 * Updates 3D Visualizatioin
 */
void update_visualization(){

        /** VISUALIZATION */
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        //RAW SINGLE VIEW DATA
        if(show_raw_data) {
                viewer->addPointCloud(cloud_trans, "view");
                //viewer->addPointCloud(cloud_corrected, "view_corrected");


                if(show_normals) {
                        viewer->addPointCloudNormals<PointType,NormalType>(cloud_trans,cloud_trans_normals);
                }
                lar_vision::display_cloud(*viewer, planes, 0,255,0, 1, "planes");

        }

        //TSDF CLUSTERS
        std::string cluster_name;
        if(show_tsdf_clusters) {
                Palette palette;
                for(int i = 0; i < tsdf_clusters.size(); i++) {
                        cluster_name = "tsdf_cluster_" + boost::lexical_cast<std::string>(i);
                        if(i<= cluster_consumed) {
                                //  display_cloud(*viewer, tsdf_clusters[i].cloud, 0, 0, 0, 4, cluster_name);
                        }else{
                                viewer->addPointCloud(tsdf_clusters[i].cloud,cluster_name);
                        }
                }
        }else{
                viewer->addPointCloud(tsdf_cloud,"tsdf");
        }


        //TARGET OBJECT
        if(show_target_object_slices) {
                draw_reference_frame(*viewer, target_object_rf, 0.1f, "target_object_rf");
                display_cloud(*viewer,target_object_approach_slice,255,255,255,4,"target_object_rf_slice");


                draw_reference_frame(*viewer,  target_object_approach_rf, 0.1f, "target_object_approach_rf");
                draw_reference_frame(*viewer,  target_object_approach_rf_wrist, 0.1f, "target_object_approach_rf_wrist");
                display_cloud(*viewer,target_grasp_configuration_cloud,255,0,255,15,"target_grasp_configuration_cloud");
        }


        //mesh



        viewer->addPolygonMesh(comau_mesh,"comau_base");

        viewer->addPolygonMesh(gripper_mesh,"gripper");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.4,0.4,0.4, "gripper");

        if(show_target_object_slices) {
                viewer->addPolygonMesh(gripper_mesh_ontarget,"gripper_ontarget");
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.8,0.2,0, "gripper_ontarget");
        }
        //pcl::PointCloud<PointType>::Ptr xxx(new pcl::PointCloud<PointType>());
        //pcl::fromPCLPointCloud2(triangles.cloud, *xxx);
        //display_cloud(*viewer,xxx,255,255,255,4,"gripper");

        std::cout << "T_O_CAMERA \n"<<T_0_CAMERA<<std::endl;
}

/**
 * Grasp Target Object full Pipe
 */
void compute_grasp_pipe(){
        clusterize_tsdf();
        slice_target_object();
        compute_target_object_approach();
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

        if(correcting_pose){
          pcl::transformPointCloud(*cloud_trans, *cloud_trans, T_CURRENTPOSE_REALPOSE);
        }

        lar_vision::compute_normals(cloud_trans,cloud_trans_normals);

        //Plane extractions
        plane_segmentation();
        //Remove all points beside highest plane
        if(segmentation_plane_success) {
                segmentation_plane_pass_filter.setInputCloud (cloud_trans);
                segmentation_plane_pass_filter.filter (*cloud_trans);
        }

        //Integrate Cloud in common TSDF
        if(data_to_consume>0) {
                integrate_current_cloud();
                data_to_consume--;
                if(data_to_consume<=0) {
                        if(consumed_data%slots==0) {
                                simple_cloud_from_tsdf_update();
                                compute_grasp_pipe();
                        }
                }
        }

        update_visualization();

}



/**
 * Robot Pose Received
 */
void pose_cb(const geometry_msgs::Pose& pose) {

        lar_tools::create_eigen_4x4_d(pose,T_BASE_ROBOT);

        T_0_ROBOT = T_0_BASE * T_BASE_ROBOT;
        T_0_CAMERA = T_0_ROBOT * T_ROBOT_CAMERA;
      //  T_0_CAMERA = T_0_CAMERA * T_CURRENTPOSE_REALPOSE;

        transform_gripper_mesh();
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
                if(robot_standing_frames==1)
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

        // ROS_INFO("Key Pressed code: %d",event.getKeyCode());
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
        if (event.getKeySym() == "n" && event.keyDown()) {
                if(tsdf_clusters.size()>0) {
                        cluster_consumed++;
                        if(cluster_consumed>tsdf_clusters.size()-1) {
                                cluster_consumed=-1;
                        }
                        ROS_INFO("Cluster consumed %d",cluster_consumed);
                        compute_grasp_pipe();
                }
                //show_normals = !show_normals;
        }
        if (event.getKeySym() == "p" && event.keyDown()) {
                if(correcting_pose) {
                        correcting_pose = false;
                        lar_tools::create_eigen_4x4_d(0,0,0, 0,0, 0, T_CURRENTPOSE_REALPOSE);
                }else{
                        correcting_pose = true;
                        current_pose_correction();
                }
        }

        if (event.getKeySym() == "j" && event.keyDown()) {
                show_tsdf_clusters = !show_tsdf_clusters;
                if(show_tsdf_clusters) ROS_INFO("Cluster view activated!");
                else ROS_INFO("Cluster view deactivated!");
        }

        if (event.getKeySym() == "m" && event.keyDown()) {
                show_target_object_slices = !show_target_object_slices;
                ROS_INFO("Show Target Object %s",show_target_object_slices ? "activated" : "deactivated");
        }
        //Approach keys
        if (event.getKeyCode() == 45 && event.keyDown()) {
                apporach_offset_distance -= 0.01;
                if(apporach_offset_distance<0.0) apporach_offset_distance=0.0;
                ROS_INFO("Approach Offset %f",apporach_offset_distance);
                compute_approach_rf_refinement();
                send_target_approach_to_robot();
        }
        if (event.getKeyCode() == 43 && event.keyDown()) {
                apporach_offset_distance += 0.01;
                ROS_INFO("Approach Offset %f",apporach_offset_distance);
                compute_approach_rf_refinement();
                send_target_approach_to_robot();
        }
        if (event.getKeySym() == "o" && event.keyDown()) {
                send_target_approach_to_robot();
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
        tf_broadcaster= new tf::TransformBroadcaster;

        //COMMUNICATION
        robot_node = new lar_tools::UDPNode("127.0.0.1",7750);
        if(!robot_node->isReady() ) {
                ROS_INFO("ROBOT NODE UDP ERROR!");
        }

        /** PARAMETERS */
        //nh->param<std::string>("folder", path, "~/temp/temp_clouds/");
        //nh->param<std::string>("output_name", output_name,"merge_results");
        nh->param<std::string>("out_folder", out_dir,"/home/daniele/temp/");

        //Camera paratemetrs
        nh->param<bool>("simple_merge", do_simple_merge, false);
        nh->param<double>("bound_x_min", bound_x_min, 0.0);
        nh->param<double>("bound_x_max", bound_x_max, 1.5);
        nh->param<double>("bound_y_min", bound_y_min, -0.4);
        nh->param<double>("bound_y_max", bound_y_max, 0.4);

        double bound_x_min = 0.0f;
        double bound_x_max = 1.5;
        double bound_y_min = -0.4;
        double bound_y_max = 0.4;

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
        nh->param<bool>("use_marching_cube", use_marching_cube, true);

        //Acquisition parameters
        nh->param<int>("slots", slots, 30);
        nh->param<double>("offx", offx, 0.0);
        nh->param<double>("offy", offy, 0.0);
        nh->param<double>("offz", offz, 0.0);
        nh->param<int>("standing_frames",auto_integrate_stading_frames_th,5);
        nh->param<bool>("auto_integrate",auto_integrate_if_robot_is_standing,false);

        //Segmentation parameters
        nh->param<double>("segmentation_plane_highest_th", segmentation_plane_highest_th, 0.03);
        nh->param<double>("segmentation_plane_slice", segmentation_plane_slice, 0.01);
        nh->param<double>("segmentation_plane_startz", segmentation_plane_startz, 0.00);
        nh->param<double>("segmentation_plane_height", segmentation_plane_height, 2.00);
        nh->param<int>("segmentation_plane_min_inliers", segmentation_plane_min_inliers, 50);
        nh->param<double>("segmentation_plane_angle_th", segmentation_plane_angle_th, 10.0);
        nh->param<int>("segmentation_plane_speedup", segmentation_plane_speedup, 1);


        //Slicing & Grasping
        nh->param<double>("grasp_object_slice_size", grasp_object_slice_size, 0.01);
        nh->param<double>("grasp_object_slice_h", grasp_object_slice_h, 0.06);
        nh->param<double>("grasp_hull_alpha",grasp_hull_alpha,0.1);
        nh->param<double>("grasp_hull_delta",grasp_hull_delta,0.01);
        nh->param<double>("grasp_min_offset",grasp_min_offset,0.01);
        nh->param<double>("grasp_max_offset",grasp_max_offset,0.2);
        nh->param<double>("grasp_min_radius",grasp_min_radius,0.01);
        nh->param<double>("grasp_max_radius",grasp_max_radius,0.05);
        nh->param<double>("grasp_fritction_cone_angle",grasp_fritction_cone_angle,M_PI/2.0);
        nh->param<double>("grasp_max_curvature",grasp_max_curvature,1);
        nh->param<double>("grasp_approach_angle_offset",grasp_approach_angle_offset,0.707);
        nh->param<int>("grasp_type",grasp_type,CRABBY_GRIPPER_STATUS_PARALLEL);



        //Compute TSDF Resolution
        int desired_res = tsdf_size / cell_size;
        int n = 1;
        while (desired_res > n) n *= 2;
        tsdf_res = n;

        /** VIEWER */
        viewer = new pcl::visualization::PCLVisualizer("viewer");
        viewer->registerKeyboardCallback(keyboardEventOccurred, (void*) &viewer);
        pcl::io::loadPolygonFileSTL("/home/daniele/gripper_full.stl", gripper_mesh);
        pcl::io::loadPolygonFileSTL("/home/daniele/gripper_full.stl", gripper_mesh_ontarget);
        pcl::io::loadPolygonFileSTL("/home/daniele/gripper_full.stl", gripper_mesh_original);

        //COMAU BASE
        {
                pcl::io::loadPolygonFileSTL("/home/daniele/comau_base.stl", comau_mesh);

                Eigen::Matrix4d comau_base;
                lar_tools::create_eigen_4x4_d(0,0,-0.4,0,0,0,comau_base);

                pcl::PointCloud<pcl::PointXYZ> cloudp2;
                pcl::fromPCLPointCloud2(comau_mesh.cloud, cloudp2);
                pcl::transformPointCloud(cloudp2, cloudp2, comau_base);
                pcl::toPCLPointCloud2(cloudp2, comau_mesh.cloud);
        }



        /** TRANSFORMS */
        lar_tools::create_eigen_4x4_d(0,0,0, 0,0, 0, T_0_BASE);
        lar_tools::create_eigen_4x4_d(0,0,0, 0,0, 0, T_CURRENTPOSE_REALPOSE);
        lar_tools::create_eigen_4x4_d(offx, offy, offz, 0,0, 0, T_WORLD_0);
        //lar_tools::create_eigen_4x4_d(0.061,-0.0094,-0.1488,179.0 * M_PI/180.0f,0,-89.5 * M_PI/180.0f,T_ROBOT_CAMERA);
        lar_tools::create_eigen_4x4_d(0,0,0,0,0,0,T_ROBOT_CAMERA);
        lar_tools::create_eigen_4x4_d(0, 0, 0, 0,0, 0, T_CAMERA_ROTATION);   //Rotate camera for Geometry Construction

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
