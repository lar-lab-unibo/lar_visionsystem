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

#include <sstream>
#include "MathUtils.h"
#include "lar_comau/ComauState.h"
#include "lar_comau/ComauCommand.h"
#include "lar_visionsystem/MarkerApproachCommand.h"

using namespace lar_visionsystem;


ros::Publisher pub;
std::string following_target = "";

bool GLOBAL_NODE_ACTIVE_STATUS = true; // TRUE IF NODE IS ACTIVE
bool GLOBAL_CALIBRATION_ACTIVE = true; // TRUE IF CALIBRATION LOOP IS ACTIVE

/** OPERATIVE MODES */
enum OpMode {
        JOYSTICK_CONTROL,
        FOLLOW_TARGET
};
OpMode current_mode = JOYSTICK_CONTROL;


/** Calibration Control */
std::string calibration_marker_name = "lar_marker_600";
tf::StampedTransform t_cam_marker;
tf::StampedTransform t_0_marker;
tf::StampedTransform t_0_6;
tf::Transform t_6_0;
tf::Transform t_marker_cam;
tf::Transform t_6_cam;


/** Follow Target Control */
int follow_target_id = -1;

/** Joystick Callback Control */
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
                MathUtils::getRotationsByName("front_90",joystick_rotation);
                MathUtils::poseToKDLRotation(joystick_set_point,joystick_rotation,true);
        }

        if(msg.buttons[5]==1 && GLOBAL_CALIBRATION_ACTIVE) {
              lar_comau::ComauCommand comau_command;
              comau_command.command = "tool";
              MathUtils::poseToTF(comau_command.pose,t_6_cam,true,1.0f);

              std::cout << comau_command <<std::endl;
              GLOBAL_NODE_ACTIVE_STATUS = false;
        }


        //CLOSE NODE WITH START BUTTON
        if(msg.buttons[9]==1) {
                GLOBAL_NODE_ACTIVE_STATUS= false;
        }
}

/** COMAU STATE */
lar_comau::ComauState comau_current_state;
bool comau_sync = false;
void comau_cb(const lar_comau::ComauState& msg){
        comau_current_state = msg;
        if(!comau_sync) {
                joystick_set_point = msg.pose;
                comau_sync = true;
        }
}

/** COMMAND PARSING SYSTEM */
std::vector<std::string> current_command_list;
void parseCommand(const std_msgs::String& msg){

        //READ COMMANDS
        current_command_list.clear();
        std::stringstream ss(msg.data);
        std::string item;
        while (std::getline(ss, item, ' ')) {
                current_command_list.push_back(item);
        }


        //CLOSE
        if(current_command_list[0]=="close") {
                GLOBAL_NODE_ACTIVE_STATUS = false;
        }

        if(current_command_list[0]=="mode") {
                if(current_command_list[1]=="joystick") {
                        current_mode = JOYSTICK_CONTROL;
                }else if(current_command_list[1]=="follow") {
                        if(current_command_list.size()>=3) {
                                int id = atoi(current_command_list[2].c_str());
                                current_mode = FOLLOW_TARGET;
                                follow_target_id = id;
                        }
                }else{

                }
        }


}

/** MAIN NODE **/
int
main (int argc, char** argv)
{
        // Initialize ROS
        ros::init (argc, argv, "comau_tf_follower");
        ros::NodeHandle nh;
        tf::TransformListener listener;
        tf::TransformBroadcaster tf_broadcaster;

        // PUB & SUB
        ros::Subscriber comau_tf_follower = nh.subscribe( "lar_visionsystem/comau_tf_follower",1,parseCommand );
        ros::Subscriber joy_sub = nh.subscribe ("joy", 1, joy_cb);
        ros::Subscriber comau_sub = nh.subscribe ("lar_comau/comau_full_state_publisher", 1, comau_cb);
        ros::Publisher comau_cartesian_controller = nh.advertise<lar_comau::ComauCommand>("lar_comau/comau_cartesian_controller", 1);
        ros::Publisher marker_approach = nh.advertise<lar_visionsystem::MarkerApproachCommand>("lar_visionsystem/marker_approach", 1);


        //Initialize Joystick
        {
                joystick_speed_max.x = joystick_speed_max.y = joystick_speed_max.z =  0.5f;
                joystick_speed.x = joystick_speed.y = joystick_speed.z = 0.0f;

                joystick_angular_speed_max.x = joystick_angular_speed_max.y = joystick_angular_speed_max.z =  M_PI/1000.0f;
                joystick_angular_speed.x = joystick_angular_speed.y = joystick_angular_speed.z = 0.0f;

                joystick_set_point.position.x = 1000.0f;
                joystick_set_point.position.y = 0.0f;
                joystick_set_point.position.z = 0.0f;

                KDL::Rotation joystick_rotation;
                MathUtils::getRotationsByName("",joystick_rotation);
                MathUtils::poseToKDLRotation(joystick_set_point,joystick_rotation,true);
        }
        //END Initialize Joystick



        tf::StampedTransform transform;
        tf::Transform current_position;
        lar_comau::ComauCommand comau_command;
        comau_command.command = "joint";

        // Spin
        while( ros::ok() && GLOBAL_NODE_ACTIVE_STATUS ) {

                if(comau_sync) {
                        std::cout << "*** SYNC WITH COMAU ACTIVE! ***"<<std::endl;
                }
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

                if(current_mode==FOLLOW_TARGET) {
                        std::cout << "Mode: Follow Target"<<std::endl;
                        std::cout << "Following target: "<<follow_target_id<<std::endl;

                        lar_visionsystem::MarkerApproachCommand approach_command;
                        approach_command.marker_id = follow_target_id;

                }


                if(GLOBAL_CALIBRATION_ACTIVE) {
                        try{
                                listener.lookupTransform("camera_rgb_frame", calibration_marker_name, ros::Time(0), t_cam_marker);
                                listener.lookupTransform("base", "comau_base_marker",ros::Time(0), t_0_marker);
                                listener.lookupTransform("base", "comau_t0U",ros::Time(0), t_0_6);

                                t_6_0 = t_0_6.inverse();
                                t_marker_cam = t_cam_marker.inverse();
                                t_6_cam = t_6_0 * t_0_marker;
                                t_6_cam = t_6_cam * t_marker_cam;

                                tf_broadcaster.sendTransform(tf::StampedTransform(t_6_cam, ros::Time::now(), "comau_t0U", "comau_t0CAM"));

                                tf::Transform full = t_0_6*t_6_cam;
                                tf_broadcaster.sendTransform(tf::StampedTransform(full, ros::Time::now(), "base", "comau_t0CAM_full"));
                        }
                        catch (tf::TransformException ex) {
                                ROS_ERROR("%s",ex.what());
                                ros::Duration(1.0).sleep();

                        }
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

                for(int i =0; i < current_command_list.size(); i++) {
                        std::cout << "Command " << i << " "<<current_command_list[i]<<std::endl;
                }

                tf_broadcaster.sendTransform(tf::StampedTransform(current_position, ros::Time(0), "base", "comau_tf_target"));
                std::cout << "Sending "<<current_position.getOrigin()[0]<<std::endl;

                MathUtils::poseToTF(comau_command.pose,current_position,true);
                std::cout << comau_command<<std::endl;
                comau_cartesian_controller.publish(comau_command);

                ros::spinOnce();
                std::system("clear");
        }
}
