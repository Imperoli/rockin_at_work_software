#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include "opencv2/core/core.hpp"
#include <math.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <boost/thread.hpp>
#include "Eigen/Dense"
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "utils.h"

using namespace cv;

mapInfo map_info;
robotInfo robot_info;

actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction>* as;
actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* holonomic_ac;
actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* move_base_ac;
std::vector<Eigen::Vector2f> checkpoints;


void computeNearestCheckpoint(Eigen::Vector2f& pos, std::vector<Eigen::Vector2f>& points, Eigen::Vector2f& nearest)
{
  float min_dist=1000;
  float dist;
  for(size_t i=0;i<points.size();i++){
    dist=(points[i]-pos).norm();
    if(dist<min_dist){ nearest=points[i]; min_dist=dist;}
  }
}

void action_cb(const move_base_msgs::MoveBaseGoalConstPtr& goal)
{
  tf::TransformListener listener;
	std::string base_frame="base_footprint";
	std::string map_frame="map";
	tf::StampedTransform transform;
	tf::Vector3 axis;

  //ros::spinOnce();
  for(int iter=0;iter<3;iter++){
    try{
      ros::Time now = ros::Time::now();
      listener.waitForTransform(map_frame, base_frame, now, ros::Duration(1));
      listener.lookupTransform(map_frame, base_frame, now, transform);
      listener.lookupTransform(map_frame, base_frame, now, transform);
      listener.lookupTransform(map_frame, base_frame, now, transform);
      float robot_posx=transform.getOrigin().x();
      float robot_posy=transform.getOrigin().y();
      float robot_orient=transform.getRotation().getAngle();
      
      Eigen::Vector2f robot_pos(robot_posx, robot_posy);
      Eigen::Vector2f target(goal->target_pose.pose.position.x, goal->target_pose.pose.position.y);
      Eigen::Vector2f checkpoint1, checkpoint2;
      computeNearestCheckpoint(robot_pos, checkpoints, checkpoint1);
      computeNearestCheckpoint(target, checkpoints, checkpoint2);
      
      move_base_msgs::MoveBaseGoal move_base_goal;
      geometry_msgs::PoseStamped target_pose;
      target_pose.header.frame_id = map_frame;
      
      target_pose.pose.position.x=checkpoint1(0);
      target_pose.pose.position.y=checkpoint1(1);
      Eigen::Vector3f check1(checkpoint1(0), checkpoint1(1), 0);  Eigen::Vector3f check2(checkpoint2(0), checkpoint2(1), 0);
      if(check2!=check1)
      {
        Eigen::Quaternionf q; q.setFromTwoVectors(Eigen::Vector3f::UnitX(), (check2-check1)/(check2-check1).norm());
        target_pose.pose.orientation.x=q.x(); target_pose.pose.orientation.y=q.y(); target_pose.pose.orientation.z=q.z(); target_pose.pose.orientation.w=q.w();
      }
      else
      {
        Eigen::Matrix3f m; m<<cos(robot_orient),-sin(robot_orient), 0, sin(robot_orient), cos(robot_orient), 0, 0, 0, 1;
        Eigen::Quaternionf q(m);
        target_pose.pose.orientation.x=q.x(); target_pose.pose.orientation.y=q.y(); target_pose.pose.orientation.z=q.z(); target_pose.pose.orientation.w=q.w();
      }
      target_pose.header.stamp = ros::Time::now();
	    move_base_goal.target_pose = target_pose;
	    holonomic_ac->sendGoal(move_base_goal);
	    holonomic_ac->waitForResult();
	    if(holonomic_ac->getState()!=actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO("cannot move the base for some reason (step 1)");
        as->setAborted();
        return ;
      }
	    
      target_pose.pose.position.x=checkpoint2(0);
      target_pose.pose.position.y=checkpoint2(1);
      target_pose.pose.orientation=goal->target_pose.pose.orientation;
      target_pose.header.stamp = ros::Time::now();
	    move_base_goal.target_pose = target_pose;
	    move_base_ac->sendGoal(move_base_goal);
	    move_base_ac->waitForResult();
	    if(move_base_ac->getState()!=actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO("cannot move the base for some reason (step 2)");
        as->setAborted();
        return ;
      }
	    
	    target_pose.pose.position.x=target(0);
      target_pose.pose.position.y=target(1);
      target_pose.pose.orientation=goal->target_pose.pose.orientation;
      target_pose.header.stamp = ros::Time::now();
	    move_base_goal.target_pose = target_pose;
	    holonomic_ac->sendGoal(move_base_goal);
	    holonomic_ac->waitForResult();
	    if(holonomic_ac->getState()!=actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO("cannot move the base for some reason (step 3)");
        as->setAborted();
        return ;
      }
	    
	    as->setSucceeded();
	    return;
    }
    catch (tf::TransformException ex){
    //  ROS_ERROR("base_plannerAction: %s",ex.what());
    //  ROS_ERROR_STREAM("TF from " << base_frame << "  to " << map_frame);
      //as->setAborted();
    }
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "hybrid_base_planner");
  ros::NodeHandle n;

  readInfoFiles(argv[1],map_info,robot_info);
  compute_checkpoints(map_info,checkpoints);
  
  holonomic_ac = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ("holonomic_goto", true);
  ROS_INFO("waiting for holonomic_goto");
  holonomic_ac->waitForServer();
  move_base_ac = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ("move_base", true);
  ROS_INFO("waiting for move_base");
  move_base_ac->waitForServer();
  ROS_INFO("OK!");
  
  as =new actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction>(n, "goto", action_cb, false);
  as->start();
  
  ros::spin();
  
  return 0; 
}

