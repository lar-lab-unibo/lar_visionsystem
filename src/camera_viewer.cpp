#include "ros/ros.h"
#include "iostream"

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <tf/transform_broadcaster.h>


static const char* DEFAULT_VIDEO_NODE_PARAMETERS_FILE = "/opt/visionSystemLegacy/data/kinect_parameters.txt";

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
        try
        {
                cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
                cv::waitKey(30);
        }
        catch (cv_bridge::Exception& e)
        {
                ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
}


/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/

int main(int argc, char **argv)
{

        std::string camera_topic_name = "/camera/rgb/image_mono";

        ros::init(argc, argv, "camera_viewer");

        ros::NodeHandle nh;

        cv::namedWindow("view");
        cv::startWindowThread();

        image_transport::ImageTransport it(nh);
        image_transport::Subscriber sub = it.subscribe(camera_topic_name.c_str(), 1, imageCallback);

        ros::spin();

        cv::destroyWindow("view");

        return 0;
} // end main()
