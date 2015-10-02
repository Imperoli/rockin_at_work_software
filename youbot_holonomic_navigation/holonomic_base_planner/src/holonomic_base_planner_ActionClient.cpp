/*
 * holonomic_base_planner_ActionClient.cpp
 *
 *      Author: Marco Imperoli
 */

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <math.h>
#include "Eigen/Dense"
#include "utils.h"

using namespace cv;

actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* ac;
Mat map;

float target_theta=0; int target_theta_deg=0;
mapInfo map_info;
robotInfo robot_info;
Mat show;

static void onMouse( int event, int x, int y, int, void* )
{
 // if( event != EVENT_LBUTTONDOWN )
 // return;
  int x2=x/map_info.map_scale;
  int y2=y/map_info.map_scale;
  float px=(float)(x2)*map_info.resolution+map_info.map_origin_x;
  float py=(float)(map.rows-1-y2)*map_info.resolution+map_info.map_origin_y;
 // std::cout<<"x "<<px<<" y "<<py<<std::endl;
 
  RotatedRect robot(Point2f(x2,y2), Size(robot_info.base_x/map_info.resolution, robot_info.base_y/map_info.resolution), -target_theta_deg);
  Point2f vertices[4];
  robot.points(vertices);
  show=map.clone();
  for (int i = 0; i < 4; i++)
    line(show, vertices[i], vertices[(i+1)%4], Scalar(80));
      
  if (map_info.map_scale>1)
    resize(show, show, cv::Size(map.cols*map_info.map_scale,map.rows*map_info.map_scale), 0, 0,cv::INTER_NEAREST);
  
  if( event == EVENT_LBUTTONDOWN )
  {
   // ac->cancelAllGoals();
    std::cout<<"x "<<px<<" y "<<py<<" GO"<<std::endl;
    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id = "map";
	  target_pose.header.stamp = ros::Time::now();
    target_pose.pose.position.x=px;
    target_pose.pose.position.y=py;
    
    Eigen::Quaternionf q(Eigen::AngleAxisf(target_theta, Eigen::Vector3f::UnitZ()));
    target_pose.pose.orientation.x = q.x();
    target_pose.pose.orientation.y = q.y();
    target_pose.pose.orientation.z = q.z();
    target_pose.pose.orientation.w = q.w();
    
    move_base_msgs::MoveBaseGoal move_base_goal;
	  move_base_goal.target_pose = target_pose;
	  
	  ac->sendGoal(move_base_goal);
	  ac->waitForResult();
  }
}

void trackbarOrientation( int, void* ){
	target_theta=target_theta_deg*M_PI/180.f;
}

int main(int argc, char **argv){
	
	ros::init(argc, argv, "goto_client");
  ros::NodeHandle nh;
  
  readInfoFiles(argv[1],map_info,robot_info);
  map = imread(map_info.file_name, CV_LOAD_IMAGE_GRAYSCALE);
	
	ac = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ("goto", true);
	ROS_INFO("waiting for goto");
  ac->waitForServer();

  namedWindow("GUI",CV_WINDOW_AUTOSIZE);
  createTrackbar("orientation", "GUI", &target_theta_deg, 360, trackbarOrientation);
  setMouseCallback( "GUI", onMouse, 0 );
  show=map;
  if (map_info.map_scale>1)
    resize(map, show, cv::Size(map.cols*map_info.map_scale,map.rows*map_info.map_scale), 0, 0,cv::INTER_NEAREST);
  while(nh.ok())
  {
    ros::spinOnce();
    cv::imshow("GUI",show);
	  cv::waitKey(10);
	}

	return 0;
}


