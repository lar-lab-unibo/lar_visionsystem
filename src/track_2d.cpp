#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>
#include <SimpleBluetoothNode.h>

namespace lar_visionsystem{
    
    /**
    * 2D Object Pose
    */
    struct Pose2D{
        double x;
        double y;
        double theta;
        
        Pose2D(){
            this->x = 0;
            this->y = 0;
            this->theta = 0;
        }
        
        Pose2D(double x, double y, double theta){
            this->x = x;
            this->y = y;
            this->theta = theta;
        }
    };
    
    /**
    * Track Object
    */
    struct Track2D{
        double max_x;
        double max_y;
        double ratio;
        std::vector<std::string> obstacles_names;
        std::vector<std::string> robots_names;
        std::vector<Pose2D> obstacles_poses;
        std::vector<Pose2D> robots_poses;
        std::vector<double> obstacles_sizes;
        std::vector<double> robots_sizes;
        std::vector<double> obstacles_height;
        std::vector<double> robots_height;
        
        Track2D(double max_x,double max_y, double ratio){
            this->max_x = max_x;
            this->max_y = max_y;
            this->ratio = ratio;
        }
    };
    
}

using namespace lar_visionsystem;




/**
* Builds Pose2D from a 3D Transform. Targets have to be to same z level, use "transform_height" otherwise
*/
void transformToPose2D(tf::Transform& transform, lar_visionsystem::Pose2D& pose, double transform_height){
    tf::Matrix3x3 m(transform.getRotation());
    double rx,ry,rz;
    m.getEulerZYX(rz,ry,rx);
    
    pose.x = transform.getOrigin()[0];
    pose.y = transform.getOrigin()[1];
    pose.theta = rz;
}





/**
* Gets Pose2D from a 3D Transform by name
*/
void getPose2D(tf::TransformListener& listener,std::string& base_frame_id,std::string& frame_id, lar_visionsystem::Pose2D& pose, double transform_height){
    
    tf::StampedTransform transform;
    try{
      listener.lookupTransform(base_frame_id, frame_id, ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      return;
    }
    
    transformToPose2D(transform,pose,0);
}

/**
* Gets Pose2D from a 3D Transform by names
*/
void getPoses2D(tf::TransformListener& listener,std::string& base_frame_id,std::vector<std::string>& frame_ids, std::vector<lar_visionsystem::Pose2D>& poses, std::vector<double>& transform_heights){
    
    
    for(int i = 0; i < frame_ids.size(); i++){
        tf::StampedTransform transform;
        
        try{
            listener.lookupTransform(base_frame_id, frame_ids[i], ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
          return;
        }

        transformToPose2D(transform,poses[i],0);
    }
    
    
}

/**
* Draw Track on image
*/
void drawTrack(
    lar_visionsystem::Track2D& track,
    cv::Mat& target){
    
    double ratio = track.ratio;
    int padding = 100;
    int w = padding*2+track.max_x * track.ratio;
    int h = padding*2+track.max_y * track.ratio;
    
    target = cv::Mat(h,w,CV_8UC3);
    target.setTo(cv::Scalar(0));
    
    int arrow_size = 50;
    cv::Point zero(padding,h-padding);
    cv::Point zero_x = zero + cv::Point(arrow_size,0);
    cv::Point zero_y = zero - cv::Point(0,arrow_size);
    
    cv::line(target, zero, zero_x, cv::Scalar(0,0,255),2);
    cv::line(target, zero, zero_y, cv::Scalar(0,255,0),2);
    
    
    for(int i = 0; i < track.obstacles_poses.size(); i++){
        double size = track.obstacles_sizes[i]*ratio;
        Pose2D p = track.obstacles_poses[i];
        cv::Point pp(p.x*ratio,-p.y*ratio);
        pp = zero + pp;
        cv::Point pp_dir(size*cos(p.theta),-size*sin(p.theta));
        pp_dir = pp + pp_dir;
      
        cv::circle(target,pp,size,cv::Scalar(0,0,255));
        cv::line(target,pp,pp_dir,cv::Scalar(255,255,0),2);
    }
    
    for(int i = 0; i < track.robots_poses.size(); i++){
        double size = track.robots_sizes[i]*ratio;
        Pose2D p = track.robots_poses[i];
        cv::Point pp(p.x*ratio,-p.y*ratio);
        pp = zero + pp;
        cv::Point pp_dir(size*cos(p.theta),-size*sin(p.theta));
        pp_dir = pp + pp_dir;
      
        cv::circle(target,pp,size,cv::Scalar(255,0,0));
        cv::line(target,pp,pp_dir,cv::Scalar(255,255,0),2);
    }
}

/**
* Multiple Bluetooth Connections
*/
void connectBluetoothNodes(std::vector<std::string>& bluetooth_nodes, std::vector<SimpleBluetoothNode>& nodes){
    for(int i = 0; i < bluetooth_nodes.size(); i++){
        SimpleBluetoothNode node(bluetooth_nodes[i]);
        std::cout << "Node: "<<bluetooth_nodes[i]<<" -> "<<node.status<<std::endl;
        nodes.push_back(node);
    }
}

/**
* Broadcast Message Send
*/
void sendMessageToBluetoothNodes(std::vector<SimpleBluetoothNode>& bluetooth_nodes,std::string message){
     for(int i = 0; i < bluetooth_nodes.size(); i++){
        bluetooth_nodes[i].writeMessage(message);
     }
}

/**
* NODE
*/
int main(int argc, char** argv){
 
    
  ros::init(argc, argv, "track_2d");

    
  ros::NodeHandle node("~");
  tf::TransformListener listener;
    
  std::string prova;
  
    
  std::string track_base_frame;
  node.getParam("base_id",track_base_frame);
  
    
  /** Bluetooth nodes **/
  std::vector<std::string> bluetooth_nodes_addresses;
  std::vector<SimpleBluetoothNode> bluetooth_nodes;
  node.getParam("bluetooth_nodes",bluetooth_nodes_addresses);
  //connectBluetoothNodes(bluetooth_nodes_addresses,bluetooth_nodes);
    
    
  std::vector<std::string> obstacles_frame_ids;
  
  std::vector<double> obstacles_sizes;
  
  std::vector<std::string> robots_frame_ids;
  std::vector<double> robots_sizes;
    
  node.getParam("obstacles",obstacles_frame_ids);
  node.getParam("obstacles_sizes",obstacles_sizes);
  
  node.getParam("robots",robots_frame_ids);
  node.getParam("robots_sizes",robots_sizes);
  
  std::cout << "Obstacles: "<<obstacles_frame_ids.size()<<std::endl;
      
  for(int i = 0; i < obstacles_frame_ids.size(); i++){
      std::cout << "O: "<<obstacles_frame_ids[i]<<" "<<obstacles_sizes[i]<<std::endl;
  }
    
  std::cout << "Robots: "<<robots_frame_ids.size()<<std::endl;
      
  for(int i = 0; i < robots_frame_ids.size(); i++){
      std::cout << "R: "<<robots_frame_ids[i]<<" "<<robots_sizes[i]<<std::endl;
  }
    
    
  std::vector<Pose2D> obstacles_poses(obstacles_frame_ids.size());
  std::vector<Pose2D> robots_poses(robots_frame_ids.size());
  std::vector<double> obstacles_heights(obstacles_frame_ids.size());
  std::vector<double> robots_heights(robots_frame_ids.size());
    
    
  Track2D track(2,1,500);
  track.obstacles_poses = obstacles_poses;
  track.robots_poses = robots_poses;
  track.obstacles_sizes = obstacles_sizes;
  track.robots_sizes = robots_sizes;
  ros::Rate rate(10.0);
    
  while (node.ok()){
   
    getPoses2D(listener,track_base_frame,obstacles_frame_ids,track.obstacles_poses,obstacles_heights);
    getPoses2D(listener,track_base_frame,robots_frame_ids,track.robots_poses,robots_heights);
   
      // Pose2D pose;
    //getPose2D(listener,track_base_frame,target_frame,pose,0);
    
    std::cout << "x: "<<track.robots_poses[0].x<< " y: "<<track.robots_poses[0].y<<" theta: "<<track.robots_poses[0].theta*180/M_PI<<std::endl;
      
      //DRAW TRACK
      cv::Mat track_img;
      drawTrack(track,track_img);
      cv::imshow("track",track_img);
      cv::waitKey(100);
    
      std::stringstream ss;
      ss << "POSES!3:P_A;0;0;20;P_B;0;0;20;P_C;0;0;0#";
      sendMessageToBluetoothNodes(bluetooth_nodes,ss.str());
      
    rate.sleep();
  }
  return 0;
};