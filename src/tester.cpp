#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <iostream>

#include "geometry_msgs/Pose.h"
#include "lar_visionsystem/MarkerApproachCommand.h"
#include <kdl/frames_io.hpp>
#include <std_msgs/String.h>

using namespace std;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "tester");

  ros::NodeHandle n;

	//ros::Publisher marker_approach = n.advertise<lar_visionsystem::MarkerApproachCommand>("lar_visionsystem/marker_approach", 1);
  ros::Publisher comau_tf_follower = n.advertise<std_msgs::String>("lar_visionsystem/comau_tf_follower", 1);

  int count = 0;
  while (ros::ok())
  {

    lar_visionsystem::MarkerApproachCommand cmd;
    std_msgs::String command_msg;
    std::string command;
    int id;

    cout << "Please enter pose value: ";
    //cin >> id;
    cin >> command;

    command_msg.data = command;
    /*
    KDL::Rotation r = KDL::Rotation::Identity();
    r.DoRotY(M_PI);
    r.GetQuaternion(
      cmd.transform.transform.rotation.x,
      cmd.transform.transform.rotation.y,
      cmd.transform.transform.rotation.z,
      cmd.transform.transform.rotation.w
    );

    cmd.transform.transform.translation.x = 0.0f;
    cmd.transform.transform.translation.y = 0.0f;
    cmd.transform.transform.translation.z = 0.5f;

    cmd.marker_id = id;

    */
    comau_tf_follower.publish(command_msg);
    ros::spinOnce();

  }


  return 0;
}
