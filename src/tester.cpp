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
#include "std_msgs/Int32MultiArray.h"
#include "sensor_msgs/JointState.h"

#define BAUDRATE B9600

//SERIAL
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


using namespace lar_visionsystem;


ros::Publisher pub;
std::string following_target = "";
tf::TransformListener* listener;
tf::TransformBroadcaster* tf_broadcaster;

bool GLOBAL_NODE_ACTIVE_STATUS = true; // TRUE IF NODE IS ACTIVE
bool GLOBAL_CALIBRATION_ACTIVE = true; // TRUE IF CALIBRATION LOOP IS ACTIVE


/** Gripper Control */
bool gripper_status = false;
std::string gripper_usb_name = "/dev/ttyUSB";
int gripper_serial = 0;
struct termios gripper_serial_cfg;
float gripper_closure_angle = 140;
float gripper_closure_speed = 0;
float gripper_closure_speed_max = 0.2f;

/** SERIAL COMMUNICATION LOOP */
void sendGripperData(){
        std::stringstream ss;
        int size = -1;
        ss.str("");
        ss << (int)(gripper_closure_angle);
        ss.seekg(0, ios::end);
        size = ss.tellg();
        write(gripper_serial, ss.str().c_str(), size);
}

void angleCommand(const sensor_msgs::JointState& msg){

        if(msg.name.size()>=1 && msg.position.size()>=1) {
                gripper_closure_angle = msg.position[0];
                ROS_INFO("LAR CRAB Angle: %f",msg.position[0]);
                sendGripperData();
        }
}


/** MAIN NODE **/
int
main (int argc, char** argv)
{
        // Initialize ROS
        ros::init (argc, argv, "comau_tf_follower");
        ros::NodeHandle nh;
        listener = new tf::TransformListener ();
        tf_broadcaster = new tf::TransformBroadcaster();

        // PUB & SUB
        ros::Subscriber gripper_sub = nh.subscribe( "lar_crab/angle_command",1,angleCommand );
        //ros::Subscriber joy_sub = nh.subscribe ("joy", 1, joy_cb);
        //  ros::Publisher comau_cartesian_controller = nh.advertise<lar_comau::ComauCommand>("lar_comau/comau_cartesian_controller", 1);

        bool status = false;
        for(int i = 0; i < 10; i++) {
                std::stringstream ss;
                ss << gripper_usb_name <<i;
                //Initialize Gripper
                gripper_serial = open(ss.str().c_str(), O_RDWR | O_NONBLOCK );
                gripper_status =gripper_serial>=0;
                if(!gripper_status) {
                        ROS_INFO("USB %d Error!",i);
                }else{
                        ROS_INFO("USB %d OK",i);
                        status = true;
                        break;
                }
        }
        if(!status){
            ROS_INFO("Closing due to USB Error!");
            return (0);
        }

        //SEtting BaudRate
        tcgetattr(gripper_serial, &gripper_serial_cfg);
        speed_t brate = BAUDRATE;
        cfsetospeed(&gripper_serial_cfg, BAUDRATE); /* baud rate */
        cfsetospeed(&gripper_serial_cfg, BAUDRATE); /* baud rate */


        //Flow control options
        gripper_serial_cfg.c_cflag &= ~PARENB;
        gripper_serial_cfg.c_cflag &= ~CSTOPB;
        gripper_serial_cfg.c_cflag &= ~CSIZE;
        gripper_serial_cfg.c_cflag |= CS8;
        gripper_serial_cfg.c_cflag &= ~CRTSCTS;
        gripper_serial_cfg.c_cflag |= CREAD | CLOCAL; // turn on READ & ignore ctrl lines
        gripper_serial_cfg.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl
        gripper_serial_cfg.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
        gripper_serial_cfg.c_oflag &= ~OPOST; // make raw

        // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
        gripper_serial_cfg.c_cc[VMIN]  = 0;
        gripper_serial_cfg.c_cc[VTIME] = 0;

        //Flush configuration
        tcsetattr(gripper_serial, TCSANOW, &gripper_serial_cfg);
        if( tcsetattr(gripper_serial, TCSAFLUSH, &gripper_serial_cfg) < 0) {
                perror("init_serialport: Couldn't set term attributes");
                return -1;
        }


        ros::Rate rate(30);
        // Spin
        while( ros::ok() && GLOBAL_NODE_ACTIVE_STATUS ) {

                ros::spinOnce();
                rate.sleep();
        }

}
