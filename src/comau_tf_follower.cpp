#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <tf/transform_listener.h>
#include "pcl_ros/transforms.h"
#include "pcl_ros/impl/transforms.hpp"
#include <opencv2/opencv.hpp>

#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>

#include "sensor_msgs/Joy.h"
#include <kdl/frames_io.hpp>

#include "MathUtils.h"

using namespace lar_visionsystem;

ros::Publisher pub;
std::string following_target = "";
bool GLOBAL_NODE_ACTIVE_STATUS = true;

/** OPERATIVE MODES */
enum OpMode {
        JOYSTICK_CONTROL,
        FOLLOW_TARGET
};
OpMode current_mode = JOYSTICK_CONTROL;


void broadcastComauTransforms(  tf::TransformBroadcaster& tf_broadcaster,geometry_msgs::Pose& sending_pose){

        //END EFFECTOR
        tf::Transform t0U;
        t0U.setOrigin( tf::Vector3(
                               sending_pose.position.x/1000.0f,
                               sending_pose.position.y/1000.0f,
                               sending_pose.position.z/1000.0f
                               ));

        tf::Quaternion q(
                sending_pose.orientation.x,
                sending_pose.orientation.y,
                sending_pose.orientation.z,
                sending_pose.orientation.w
                );

        q.normalize();

        t0U.setRotation(q);
        tf_broadcaster.sendTransform(tf::StampedTransform(t0U, ros::Time::now(), "base", "target"));

}



void followTargetChange(const std_msgs::String& msg){
        following_target = msg.data;
}


/** Joystick Callback */
geometry_msgs::Pose joystick_set_point;
geometry_msgs::Vector3 joystick_speed_max;
geometry_msgs::Vector3 joystick_speed;
geometry_msgs::Vector3 joystick_angular_speed;
geometry_msgs::Vector3 joystick_angular_speed_max;

sensor_msgs::Joy current_joystick_message;
void joy_cb(const sensor_msgs::Joy& msg){

        current_joystick_message = msg;

        joystick_speed.x =  msg.axes[1]*joystick_speed_max.x;
        joystick_speed.y =  msg.axes[0]*joystick_speed_max.x;
        joystick_speed.z =  msg.axes[3]*joystick_speed_max.x;

        //X AXIS ROTATION L2/R2
        if(msg.buttons[6]==1) {
                joystick_angular_speed.x =  joystick_angular_speed_max.x;
        }else if(msg.buttons[7]==1) {
                joystick_angular_speed.x = -joystick_angular_speed_max.x;
        }else{
                joystick_angular_speed.x =  0;
        }

        //Y AXIS ROTATION A/Y
        if(msg.buttons[1]==1) {
                joystick_angular_speed.y =  joystick_angular_speed_max.y;
        }else if(msg.buttons[3]==1) {
                joystick_angular_speed.y = -joystick_angular_speed_max.y;
        }else{
                joystick_angular_speed.y =  0;
        }

        //Z AXIS ROTATION X/B
        if(msg.buttons[0]==1) {
                joystick_angular_speed.z =  joystick_angular_speed_max.z;
        }else if(msg.buttons[2]==1) {
                joystick_angular_speed.z = -joystick_angular_speed_max.z;
        }else{
                joystick_angular_speed.z =  0;
        }

        //RESET orientation
        if(msg.buttons[10]==1 && msg.buttons[11]==1) {
          KDL::Rotation joystick_rotation;
          MathUtils::getRotationsByName("",joystick_rotation);
          MathUtils::poseToKDLRotation(joystick_set_point,joystick_rotation,true);
        }

        //CLOSE NODE WITH START BUTTON
        if(msg.buttons[9]==1) {
                GLOBAL_NODE_ACTIVE_STATUS= false;
        }
}

int
main (int argc, char** argv)
{
        // Initialize ROS
        ros::init (argc, argv, "comau_tf_follower");
        ros::NodeHandle nh;
        tf::TransformListener listener;
        tf::TransformBroadcaster tf_broadcaster;

        ros::Subscriber marker_approach = nh.subscribe( "lar_visionsystem/comau_tf_follower",1,followTargetChange );
        ros::Subscriber joy_sub = nh.subscribe ("joy", 1, joy_cb);


        //Initialize Joystick
        joystick_speed_max.x = joystick_speed_max.y = joystick_speed_max.z =  0.1f;
        joystick_speed.x = joystick_speed.y = joystick_speed.z = 0.0f;

        joystick_angular_speed_max.x = joystick_angular_speed_max.y = joystick_angular_speed_max.z =  M_PI/1000.0f;
        joystick_angular_speed.x = joystick_angular_speed.y = joystick_angular_speed.z = 0.0f;


        joystick_set_point.position.x = 1000.0f;
        joystick_set_point.position.y = 0.0f;
        joystick_set_point.position.z = 0.0f;

        KDL::Rotation joystick_rotation;
        MathUtils::getRotationsByName("",joystick_rotation);
        MathUtils::poseToKDLRotation(joystick_set_point,joystick_rotation,true);



        tf::StampedTransform transform;

        tf::Transform current_position;
        // Spin
        while( ros::ok() && GLOBAL_NODE_ACTIVE_STATUS ) {

                if(current_mode==JOYSTICK_CONTROL) {
                        std::cout << "Mode: Joystick Control"<<std::endl;
                        std::cout << joystick_angular_speed<<std::endl;

                        joystick_set_point.position.x +=  joystick_speed.x;
                        joystick_set_point.position.y +=  joystick_speed.y;
                        joystick_set_point.position.z +=  joystick_speed.z;

                        KDL::Rotation joystick_rotation;
                        MathUtils::poseToKDLRotation(joystick_set_point,joystick_rotation);
                        joystick_rotation.DoRotX(joystick_angular_speed.x);
                        joystick_rotation.DoRotY(joystick_angular_speed.y);
                        joystick_rotation.DoRotZ(joystick_angular_speed.z);
                        MathUtils::poseToKDLRotation(joystick_set_point,joystick_rotation,true);

                        MathUtils::poseToTF(joystick_set_point,current_position);
                }
                /**
                    if(following_target=="") {
                            std::cout << "No target yet!"<<std::endl;
                    }else{
                            try{
                                    listener.lookupTransform("camera_rgb_frame", following_target, ros::Time(0), transform);
                                    std::cout << "Target got!"<<std::endl;

                            }
                            catch (tf::TransformException ex) {
                                    ROS_ERROR("%s",ex.what());
                                    ros::Duration(1.0).sleep();

                            }
                    }
                 */

                tf_broadcaster.sendTransform(tf::StampedTransform(current_position, ros::Time(0), "base", "comau_tf_target"));
                std::cout << "Sending "<<current_position.getOrigin()[0]<<std::endl;
                ros::spinOnce();
                std::system("clear");
        }
}
