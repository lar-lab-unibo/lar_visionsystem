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
static const char* DEFAULT_VIDEO_NODE_PARAMETERS_FILE = "/opt/visionSystemLegacy/data/kinect_parameters.txt";


/**
    Parameters
*/

//std::string camera_topic_name = "/usb_cam/image_raw";///camera/rgb/image_mono";
//std::string camera_info_file_name = "/home/daniele/catkin_ws/src/lar_visionsystem/data/lifeCamera640x480.yml";
std::string  camera_topic_name = "/camera/rgb/image_mono";
std::string  camera_info_file_name = "/home/daniele/catkin_ws/src/lar_visionsystem/data/calibrations/kinect.yml";
   

ros::NodeHandle* nh = NULL;

/**
    Aruco Detectors
*/
aruco::CameraParameters camera_parameters;
aruco::MarkerDetector marker_detector;
float marker_size = 0.1;
vector<aruco::Marker> markers_list;
vector<aruco::Marker> filtered_markers_list;
std::map<int, tf::Transform> filtered_markers_tf;

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
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat source = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::blur( source, source, cv::Size(3,3));
    marker_detector.detect(source, markers_list, camera_parameters, marker_size, false);
    
    float alpha = 0.11f;
  
    for (unsigned int i = 0; i < markers_list.size(); i++) {
        cv::Mat T = lar_visionsystem::MathUtils::getTMarker(markers_list[i]);
        std::cout <<"\n"<<markers_list[i].id<<":  "<<  T.at<float>(0,2)<<","<< T.at<float>(1,2)<<","<< T.at<float>(2,2)<<"\n";
        tf::Transform tf = lar_visionsystem::MathUtils::matToTF(T);
        
        std::map<int, tf::Transform>::iterator iter = filtered_markers_tf.find(markers_list[i].id);
        if(iter != filtered_markers_tf.end()){
            tf::Transform ftf = iter->second;

            lowPassFilter(tf,ftf,ftf,alpha);
            
            filtered_markers_tf[markers_list[i].id] = ftf;
        }else{
            filtered_markers_tf[markers_list[i].id] = tf;
        }
        
    }
    
    for (unsigned int i = 0; i < markers_list.size(); i++) {
     markers_list[i].draw(source, cv::Scalar(0, 255, 0), 2);
     aruco::CvDrawingUtils::draw3dAxis(source, markers_list[i], camera_parameters);
    }
    
    cv::imshow("view", source);
    cv::waitKey(1000/30);
    
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
    //nh->setParam("/marker_detector/lowPassAlpha", 0.1);
    
    //ros::spin();
    tf::TransformBroadcaster br;
    
    
    cv::namedWindow("view");
    cv::startWindowThread();
    
    while(nh->ok()){
        
    std::map<int, tf::Transform>::iterator iter ;
    for (iter = filtered_markers_tf.begin(); iter != filtered_markers_tf.end(); iter++) {
           
        std::stringstream ss;
        ss << "lar_marker_"<<iter->first;
        //std::cout << ss.str()<<std::endl;
        
        br.sendTransform(tf::StampedTransform(iter->second, ros::Time::now(),  "camera_rgb_frame",ss.str().c_str()));
       
    }
  
    ros::spinOnce();
        
    }
    cv::destroyWindow("view");

    return 0;
} // end main()
