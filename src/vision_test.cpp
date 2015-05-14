#include "ros/ros.h"
#include "iostream"
//#include "aruco/aruco.h"
//#include "VisionSystemLegacy/PrimeSensor.h"

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <tf/transform_broadcaster.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include <pcl/visualization/pcl_visualizer.h>
#include "/opt/ros/groovy/stacks/lsd_slam/lsd_slam_viewer/msg_gen/cpp/include/lsd_slam_viewer/keyframeMsg.h"

#include "sophus/sim3.hpp"

static const char* DEFAULT_VIDEO_NODE_PARAMETERS_FILE = "/opt/visionSystemLegacy/data/kinect_parameters.txt";

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


/**bool add(lar_visionsystem::MarkerImageService::Request  &req,
  lar_visionsystem::MarkerImageService &res)
{

     return true;
}*/

struct InputPointDense
{
	float idepth;
	float idepth_var;
	uchar color[4];
};

int width = 640;
int height = 480;
pcl::visualization::PCLVisualizer * viewer;

int frame_counter = 0;
void frameCb(lsd_slam_viewer::keyframeMsgConstPtr msg)
{
  int size = width*height;
  InputPointDense* originalInput = new InputPointDense[width*height];

  memcpy(originalInput, msg->pointcloud.data(), width*height*sizeof(InputPointDense));

  std::cout << "Copied"<<std::endl;

  Sophus::Sim3f camToWorld;
  memcpy(camToWorld.data(), msg->camToWorld.data(), 7*sizeof(float));


  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());

  cloud->width = width;
  cloud->height = height;

  float fx = msg->fx;
  float fy = msg->fy;
  float cx = msg->cx;
  float cy = msg->cy;

  float fxi = 1/fx;
  float fyi = 1/fy;
  float cxi = -cx / fx;
  float cyi = -cy / fy;

  float scale = 10.0f;

  for(int x = 0; x < width; x++){
    for(int y = 0; y < height; y++){
      float depth = 1 / originalInput[x+y*width].idepth;

      pcl::PointXYZRGBA point;

      Sophus::Vector3f pt = camToWorld * (Sophus::Vector3f((x*fxi + cxi), (y*fyi + cyi), 1) * depth);


      point.x = pt[0]*scale;
			point.y = pt[1]*scale;
			point.z = pt[2]*scale;

			point.a = 100;
      point.r = originalInput[x+y*width].color[0];
      point.g = originalInput[x+y*width].color[1];
      point.b = originalInput[x+y*width].color[2];
      cloud->points.push_back(point);


    }
  }
  std::stringstream ss;
  ss << "cloud_"<<frame_counter++;
  viewer->addPointCloud(cloud,ss.str());


}

/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/

int main(int argc, char **argv)
{

  viewer = new pcl::visualization::PCLVisualizer("ciao");
  // Set up ROS.
  ros::init(argc, argv, "camera_info_test");
  ros::NodeHandle n;

  //ros::ServiceServer service = n.advertiseService("lar_visionsystem/test_service", add);
  //ROS_INFO("Ready to add two ints.");
  //ros::spin();

//  image_transport::ImageTransport it(n);
//  image_transport::Publisher pub = it.advertise("lar_visionsystem/test_image", 1);

  //CAMERA INFO
  ros::Publisher pub_info_left = n.advertise<sensor_msgs::CameraInfo>("lar_camera_info", 1);
  sensor_msgs::CameraInfo info_left;

  ros::Subscriber keyFrames_sub = n.subscribe(n.resolveName("lsd_slam/keyframes"),20, frameCb);


  info_left.width = 640;
  info_left.height = 480;
  info_left.distortion_model = "plumb_bob";
  //info_left.D = new std::vector<float>();
  info_left.D.push_back(6.2366986890046598e-02);
  info_left.D.push_back(-2.7279126825695826e-01);
  info_left.D.push_back(8.0736432261820177e-04);
  info_left.D.push_back(-1.5164432286900785e-03);
  info_left.D.push_back(2.1905717733802280e-01);

  info_left.K[0] = 5.1344124775729779e+02f;
  info_left.K[1] =  0.0f;
  info_left.K[2] =  3.1317224272165350e+02f;
  info_left.K[3] =  0.0f;
  info_left.K[4] =        5.1340620556577676e+02f;
  info_left.K[5] =  2.3985607634257391e+02f;
  info_left.K[6] =  0.0f;
  info_left.K[7] =  0.0f;
  info_left.K[8] =  1.0f;



  aruco::CameraParameters CamParam;
    aruco::MarkerDetector MDetector;
    float MarkerSize = 0.1;
    vector<aruco::Marker> Markers;

   PrimeSensor sensor("/opt/visionSystemLegacy/data/primesense_config.xml");
   sensor.loadConfiguration("/opt/visionSystemLegacy/data/kinect.yml");
   sensor.loadParameters(DEFAULT_VIDEO_NODE_PARAMETERS_FILE);
   CamParam.readFromXMLFile("/opt/visionSystemLegacy/data/kinect.yml");

   if (!sensor.isReady()) {
       cout << "Prime Sensor not ready!";
       return 0;
   }

   tf::TransformBroadcaster br;
   tf::Transform transform;

  while (n.ok())
  {

    cv::Mat* img = sensor.grabFrameRGB();

    MDetector.detect(*img, Markers, CamParam, MarkerSize, false);
    for (unsigned int i = 0; i < Markers.size(); i++) {
      Markers[i].draw(*img, Scalar(0, 255, 0), 2);
      aruco::CvDrawingUtils::draw3dAxis(*img, Markers[i], CamParam);
      cv::Mat T = MathUtils::getTMarker(Markers[i]);
      std::cout << T <<std::endl;
      tf::Transform tf = matToTF(T);
      //std::cout << tf << std::endl;
      std::stringstream ss;
      ss << "marker_"<<Markers[i].id;
      std::cout << "PUB TF: "<<ss.str().c_str()<<std::endl;
      br.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "world", ss.str().c_str()));
    }


    sensor_msgs::ImagePtr msg;
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", *img).toImageMsg();
    //pub.publish(msg);

    info_left.header.stamp = ros::Time::now();
    pub_info_left.publish(info_left);

    //sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", *img).toImageMsg();
    //cv::imshow("Ciao",*img);
    //cv::waitKey(10);
    ros::spinOnce();

    loop_rate.sleep();
    delete img;
  }

  /**


  ros::Rate loop_rate(5);
  while(n.ok() && !viewer->wasStopped()){

    info_left.header.stamp = ros::Time::now();
    pub_info_left.publish(info_left);
    //std::cout <<info_left.header.stamp<<"\n";
    //cv::waitKey(10);
    viewer->spinOnce();
    ros::spinOnce();

    loop_rate.sleep();

  }
  *//
  return 0;
} // end main()
