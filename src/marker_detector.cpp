#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>

#include "iostream"
#include "aruco/aruco.h"

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <tf/transform_broadcaster.h>

#include "MathUtils.h"
#include <dynamic_reconfigure/server.h>

#include "lar_visionsystem/MarkerApproachCommand.h"
#include "std_msgs/Int32MultiArray.h"

static const char* DEFAULT_VIDEO_NODE_PARAMETERS_FILE = "/opt/visionSystemLegacy/data/kinect_parameters.txt";


/**
    Parameters
 */

//std::string camera_topic_name = "/usb_cam/image_raw";///camera/rgb/image_mono";
//std::string camera_info_file_name = "/home/daniele/catkin_ws/src/lar_visionsystem/data/lifeCamera640x480.yml";
std::string camera_topic_name = "/camera/rgb/image_mono";
std::string camera_info_file_name = "/home/daniele/catkin_ws/src/lar_visionsystem/data/calibrations/kinect.yml";


ros::NodeHandle* nh = NULL;
tf::TransformBroadcaster* br;

/**
    Aruco Detectors
 */
aruco::CameraParameters camera_parameters;
aruco::MarkerDetector marker_detector;
float marker_size = 0.1;
vector<aruco::Marker> markers_list;
vector<aruco::Marker> filtered_markers_list;
std::map<int, tf::Transform> filtered_markers_tf;
std::map<int, tf::Transform> approach_markers_tf;
int current_approach_target = -1;

void lowPassFilter(tf::Transform& newTF, tf::Transform& oldTF,tf::Transform& targetTF, float alpha){
        for(int i = 0; i < 3; i++)
                targetTF.getOrigin()[i] = (1.0f-alpha)*oldTF.getOrigin()[i] + alpha*newTF.getOrigin()[i];

        tf::Quaternion newQ = newTF.getRotation();
        tf::Quaternion oldQ = oldTF.getRotation();
        tf::Quaternion targetQ = targetTF.getRotation();

        for(int i = 0; i < 4; i++)
                targetQ[i] = (1.0f-alpha)*oldQ[i] + alpha*newQ[i];

        targetTF.setRotation(targetQ);
}

/**
   Image Callback
 */
cv::Mat current_image;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
        cv::Mat source = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::blur( source, source, cv::Size(3,3));
        marker_detector.detect(source, markers_list, camera_parameters, marker_size, false);

        float alpha = 0.99f;
        //float alpha = 0.11f;

        for (unsigned int i = 0; i < markers_list.size(); i++) {
                cv::Mat T = lar_visionsystem::MathUtils::getTMarker(markers_list[i]);
                std::cout <<"\n"<<markers_list[i].id<<":  "<<  T.at<float>(0,2)<<","<< T.at<float>(1,2)<<","<< T.at<float>(2,2)<<"\n";
                tf::Transform tf = lar_visionsystem::MathUtils::matToTF(T);

                std::map<int, tf::Transform>::iterator iter = filtered_markers_tf.find(markers_list[i].id);
                if(iter != filtered_markers_tf.end()) {
                        tf::Transform ftf = iter->second;

                        lowPassFilter(tf,ftf,ftf,alpha);

                        filtered_markers_tf[markers_list[i].id] = ftf;
                }else{
                        filtered_markers_tf[markers_list[i].id] = tf;
                }

        }

        for (unsigned int i = 0; i < markers_list.size(); i++) {
                if(current_approach_target==markers_list[i].id) {
                        markers_list[i].draw(source, cv::Scalar(255, 255, 0), 6,true);
                }else{
                        markers_list[i].draw(source, cv::Scalar(0, 255, 0), 2,true);
                }
                aruco::CvDrawingUtils::draw3dAxis(source, markers_list[i], camera_parameters);
        }

        cv::flip(source,source,-1);
        cv::imshow("view", source);
        current_image = source;
        cv::waitKey(1000/30);
}


void
targetMarkerReceived(const lar_visionsystem::MarkerApproachCommand& command){
        if(command.update_target_only) {
                current_approach_target = command.marker_id;
        }else{
                tf::Transform transform;

                transform.setOrigin(tf::Vector3(
                                            command.pose.position.x,
                                            command.pose.position.y,
                                            command.pose.position.z
                                            ));

                tf::Quaternion q = tf::Quaternion(
                        command.pose.orientation.x,
                        command.pose.orientation.y,
                        command.pose.orientation.z,
                        command.pose.orientation.w
                        );
                transform.setRotation(q.normalized());

                approach_markers_tf[command.marker_id] = transform;
        }
}


/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/

int main(int argc, char **argv)
{
        /** Camera Parameters */
        camera_parameters.readFromXMLFile(camera_info_file_name);



        ros::init(argc, argv, "marker_detector");
        nh = new ros::NodeHandle();


        image_transport::ImageTransport it(*nh);
        image_transport::Subscriber sub = it.subscribe(camera_topic_name.c_str(), 1, imageCallback);
        image_transport::Publisher image_publisher = it.advertise("lar_visionsystem/marker_detector_feedback", 1);
        ros::Publisher marker_list_publisher = nh->advertise<std_msgs::Int32MultiArray>("lar_visionsystem/markers_list", 1);

        ros::Subscriber marker_approach = nh->subscribe( "lar_visionsystem/marker_approach",1,targetMarkerReceived );

        br = new tf::TransformBroadcaster();

        //nh->setParam("/marker_detector/lowPassAlpha", 0.1);

        //ros::spin();



        cv::namedWindow("view");
        cv::startWindowThread();

        std_msgs::Int32MultiArray marker_list;

        while(nh->ok()) {

                std::map<int, tf::Transform>::iterator iter;

                std::stringstream ss;
                std::stringstream ss2;
                marker_list.data.clear();
                for (iter = filtered_markers_tf.begin(); iter != filtered_markers_tf.end(); iter++) {

                        marker_list.data.push_back(iter->first);

                        ss.str(""); 
                        ss << "lar_marker_"<<iter->first;
                        //std::cout << ss.str()<<std::endl;

                        br->sendTransform(tf::StampedTransform(iter->second, ros::Time(0),  "camera_rgb_frame",ss.str().c_str()));

                        if(iter->first==current_approach_target) {
                                ss2.str("");
                                ss2 << "lar_marker_approach_"<<current_approach_target;
                                br->sendTransform(tf::StampedTransform(approach_markers_tf[current_approach_target], ros::Time(0),  ss.str().c_str(),ss2.str().c_str()));
                        }

                }

                /*std::map<int, tf::Transform>::iterator iter_approach;
                   for (iter_approach = approach_markers_tf.begin(); iter_approach != approach_markers_tf.end(); iter_approach++) {

                        ss.str("");
                        ss2.str("");
                        ss << "lar_marker_"<<iter_approach->first;
                        ss2 << "lar_marker_approach_"<<iter_approach->first;

                        //std::cout << ss.str()<<std::endl;

                        br->sendTransform(tf::StampedTransform(iter_approach->second, ros::Time(0),  ss.str().c_str(),ss2.str().c_str()));
                        break;
                   }*/

                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", current_image).toImageMsg();
                image_publisher.publish(msg);
                marker_list_publisher.publish(marker_list);

                ros::spinOnce();
                std::system("clear");

        }
        cv::destroyWindow("view");

        return 0;
} // end main()
