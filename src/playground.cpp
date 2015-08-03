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
#include "geometry_msgs/Pose.h"

ros::Publisher pub;
pcl::visualization::PCLVisualizer* viewer;
std::string save_folder = "";
int save_counter = 0;

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


unsigned int text_id = 0;
bool capture = false;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
        if (event.getKeySym () == "v" && event.keyDown ())
        {
                std::cout << "r was pressed => removing all text" << std::endl;
                capture = true;
        }
}


typedef pcl::PointXYZRGBA PointType;
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
        pcl::PointCloud<PointType>::Ptr cloud_trans_filtered(new  pcl::PointCloud<PointType>);
        pcl::PCLPointCloud2 pcl_pc;
        pcl_conversions::toPCL(*input, pcl_pc);
        pcl::fromPCLPointCloud2(pcl_pc, *cloud);




        pass.setInputCloud (cloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0.0, 1.3);
        //pass.setFilterLimitsNegative (true);
        pass.filter (*cloud);

        Eigen::Matrix4f mat;
        mat << 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1;
        poseToEigen(current_comau_state,mat);

        pcl::transformPointCloud(*cloud,*cloud_trans,mat);

        sor.setInputCloud (cloud_trans);
        sor.setLeafSize (0.02f, 0.02f, 0.02f);
        sor.filter (*cloud_trans_filtered);

        if(current_comau_state.moving==false) {
                capture=false;
                if(cloud->points.size()>1000){
                //(*cloud_full) += (*cloud_trans);

                std::ofstream myfile;
                std::stringstream ss;
                ss << save_folder << "/"<<save_counter<<".txt";

                myfile.open (ss.str().c_str());
                myfile << mat;
                myfile.close();


                ss.str("");
                ss << save_folder << "/" << save_counter<<".pcd";
                pcl::io::savePCDFileASCII (ss.str().c_str(), *cloud);

                std::cout << "Saved snapshot: "<<save_counter<<std::endl;
                save_counter++;
              }
        }
        /*
           (*cloud_full) += (*cloud_trans);


           sor.setInputCloud (cloud_full);
           sor.setLeafSize (0.01f, 0.01f, 0.01f);
           sor.filter (*cloud_full_filtered);


         */
        viewer->removeAllPointClouds();
        viewer->addPointCloud(cloud_trans, "scene");
      //viewer->addPointCloud(cloud_full, "full");

        //cloud_full = cloud_full_filtered;


}

int
main (int argc, char** argv)
{
        // Initialize ROS
        ros::init (argc, argv, "my_pcl_tutorial");
        ros::NodeHandle nh;


        std::stringstream ss;
        ss << "/home/daniele/temp/"<<ros::Time::now();
        save_folder = ss.str();

        boost::filesystem::create_directory(save_folder);

        std::cout << "DIR:" <<ss.str()<<std::endl;

        viewer = new pcl::visualization::PCLVisualizer("viewer");
        viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);

        // Create a ROS subscriber for the input point cloud
        //ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, cloud_cb);
        ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);

        ros::Subscriber comau_sub = nh.subscribe ("lar_comau/comau_full_state_publisher", 1, comau_cb);

        // Spin
        while(nh.ok() && !viewer->wasStopped()) {


                viewer->spinOnce();
                ros::spinOnce();
        }
}
