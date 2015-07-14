#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <tf/transform_listener.h>
#include "pcl_ros/transforms.h"
#include "pcl_ros/impl/transforms.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <tf/transform_broadcaster.h>
#include "tf_conversions/tf_kdl.h"

#include "lar_comau/ComauState.h"
#include "lar_comau/ComauCommand.h"
#include "sensor_msgs/Joy.h"

ros::Publisher pub;
pcl::visualization::PCLVisualizer* viewer;

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

        for(int i = 0; i < 3; i++) {
                for(int j = 0; j < 3; j++) {
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

void poseToEigen(lar_comau::ComauState& msg, Eigen::Matrix4f& m){
        m(0,3) = msg.pose.position.x/1000.0f;
        m(1,3) = msg.pose.position.y/1000.0f;
        m(2,3) = msg.pose.position.z/1000.0f;
        m(3,3) = 1.0f;

        tf::Quaternion q(
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w
                );
        tf::Matrix3x3 rot(q);

        for(int i = 0; i < 3; i++) {
                for(int j = 0; j < 3; j++) {
                        m(i,j) = rot[i][j];
                }
        }

}

lar_comau::ComauState current_comau_state;
void comau_cb(const lar_comau::ComauState& msg){

        current_comau_state = msg;

}

typedef pcl::PointXYZ PointType;
pcl::PointCloud<PointType>::Ptr cloud_full(new  pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloud_full_filtered(new  pcl::PointCloud<PointType>);
pcl::VoxelGrid<PointType> sor;
pcl::PassThrough<PointType> pass;

bool shoot = false;
void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
        // Create a container for the data.
        sensor_msgs::PointCloud2 output;

        pcl::PointCloud<PointType>::Ptr cloud(new  pcl::PointCloud<PointType>);
        pcl::PointCloud<PointType>::Ptr cloud_trans(new  pcl::PointCloud<PointType>);
        pcl::PCLPointCloud2 pcl_pc;
        pcl_conversions::toPCL(*input, pcl_pc);
        pcl::fromPCLPointCloud2(pcl_pc, *cloud);


        if(shoot) {
                shoot = false;

                pass.setInputCloud (cloud);
                pass.setFilterFieldName ("z");
                pass.setFilterLimits (0.0, 1.3);
                //pass.setFilterLimitsNegative (true);
                pass.filter (*cloud);

                Eigen::Matrix4f mat;
                mat << 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
                poseToEigen(current_comau_state,mat);
                std::cout << mat <<std::endl;

                pcl::transformPointCloud(*cloud,*cloud_trans,mat);

                sor.setInputCloud (cloud_trans);
                sor.setLeafSize (0.01f, 0.01f, 0.01f);
                sor.filter (*cloud_trans);


                (*cloud_full) += (*cloud_trans);


                sor.setInputCloud (cloud_full);
                sor.setLeafSize (0.01f, 0.01f, 0.01f);
                sor.filter (*cloud_full_filtered);



                viewer->removeAllPointClouds();
                viewer->addPointCloud(cloud_full_filtered, "scene");

                cloud_full = cloud_full_filtered;

                std::cout << "Received: "<<cloud_full->points.size()<<std::endl;
        }
/*


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

        //sensor_msgs::PointCloud2 out = *input;
        //out.header.frame_id = "camera_link";
        // pub.publish(out);


}


ros::Publisher comau_cartesian_controller;

void joy_cb(const sensor_msgs::Joy& msg){
        //std::cout << msg << std::endl;

        float vel = 200.0f;
        float angular_vel = M_PI/10.0f;

        lar_comau::ComauCommand c;
        c.command = "joint";
        c.pose = current_comau_state.pose;
        c.pose.position.y += msg.axes[0]*vel;
        c.pose.position.x += msg.axes[1]*vel;
        c.pose.position.z += msg.axes[3]*vel;

        if(msg.buttons[8]==1) {
                KDL::Rotation rot = KDL::Rotation::RPY(0,M_PI,0);

                double qx,qy,qz,qw;
                rot.GetQuaternion(qx,qy,qz,qw);
                c.pose.orientation.x = qx;
                c.pose.orientation.y = qy;
                c.pose.orientation.z = qz;
                c.pose.orientation.w = qw;
        }else{
                KDL::Rotation qr = KDL::Rotation::Quaternion(
                        c.pose.orientation.x,
                        c.pose.orientation.y,
                        c.pose.orientation.z,
                        c.pose.orientation.w
                        );


                if(msg.buttons[6]==1) {
                        qr.DoRotX(angular_vel);
                }else if(msg.buttons[7]==1) {
                        qr.DoRotX(-angular_vel);
                }

                if(msg.buttons[1]==1) {
                        qr.DoRotY(-angular_vel);
                }else if(msg.buttons[3]==1) {
                        qr.DoRotY(angular_vel);
                }


                double qx,qy,qz,qw;
                qr.GetQuaternion(qx,qy,qz,qw);
                c.pose.orientation.x = qx;
                c.pose.orientation.y = qy;
                c.pose.orientation.z = qz;
                c.pose.orientation.w = qw;
        }

        if(msg.buttons[5]==1) {
              shoot = true;
        }

        comau_cartesian_controller.publish(c);
}

int
main (int argc, char** argv)
{
        // Initialize ROS
        ros::init (argc, argv, "my_pcl_tutorial");
        ros::NodeHandle nh;

        viewer = new pcl::visualization::PCLVisualizer("viewer");

        // Create a ROS subscriber for the input point cloud
        ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);
        ros::Subscriber comau_sub = nh.subscribe ("lar_comau/comau_full_state_publisher", 1, comau_cb);
        ros::Subscriber joy_sub = nh.subscribe ("joy", 1, joy_cb);
        comau_cartesian_controller = nh.advertise<lar_comau::ComauCommand>("lar_comau/comau_cartesian_controller", 1);
        //pub = nh.advertise<sensor_msgs::PointCloud2> ("mycloud", 1);

        tf::TransformListener listener;

        tf::StampedTransform t_cam_marker;
        tf::StampedTransform t_0_marker;
        tf::StampedTransform t_0_6;

        tf::Transform t_6_0;
        tf::Transform t_marker_cam;

        tf::Transform t_6_cam;

        tf::TransformBroadcaster br;

        // Spin
        while(nh.ok() && !viewer->wasStopped()) {
                viewer->spinOnce();
                ros::spinOnce();
        }
}
