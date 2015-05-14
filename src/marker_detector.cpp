#include "ros/ros.h"
#include "iostream"
#include "aruco/aruco.h"

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <tf/transform_broadcaster.h>

#include "MathUtils.h"

static const char* DEFAULT_VIDEO_NODE_PARAMETERS_FILE = "/opt/visionSystemLegacy/data/kinect_parameters.txt";

/**
    Parameters
*/
std::string camera_topic_name = "/camera/rgb/image_mono";
//TODO: Parametrizzare da topic o da arg questo file
std::string camera_info_file_name = "/home/daniele/catkin_ws/src/lar_visionsystem/data/kinect.yml";

/**
    Aruco Detectors
*/
aruco::CameraParameters camera_parameters;
aruco::MarkerDetector marker_detector;
float marker_size = 0.1;
vector<aruco::Marker> markers_list;

/**
    MAT4x4 to TF
*/
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

  for(int i = 0; i < 3; i++){
    for(int j = 0; j < 3; j++){
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

/**
Image Callback
*/
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	    
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::Mat new_image = image.clone();
    int beta = 0;
    double alpha = 1.0;
	for( int y = 0; y < image.rows; y++ )
    { for( int x = 0; x < image.cols; x++ )
         { for( int c = 0; c < 3; c++ )
              {
      new_image.at<Vec3b>(y,x)[c] =
         saturate_cast<uchar>( alpha*( image.at<Vec3b>(y,x)[c] ) + beta );
             }
    }
    }    
  
    cv::Mat source = new_image;
    
    marker_detector.detect(source, markers_list, camera_parameters, marker_size, false);
    
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
   
    
    ros::init(argc, argv, "camera_viewer");
    
    ros::NodeHandle nh;
    cv::namedWindow("view");
    cv::startWindowThread();
    
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(camera_topic_name.c_str(), 1, imageCallback);
    
    //ros::spin();
    tf::TransformBroadcaster br;
    
    while(nh.ok()){
        
        for (unsigned int i = 0; i < markers_list.size(); i++) {
 
        cv::Mat T = MathUtils::getTMarker(markers_list[i]);
        
        tf::Transform tf = matToTF(T);
        std::cout << T<<std::endl;

        std::stringstream ss;
        ss << "lar_marker_"<<markers_list[i].id;
        std::cout << ss.str()<<std::endl;

        br.sendTransform(tf::StampedTransform(tf, ros::Time::now(),  "camera_depth_frame",ss.str().c_str()));

    }
        
        ros::spinOnce();
        
    }
    cv::destroyWindow("view");

    return 0;
} // end main()
