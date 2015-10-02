#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include "youbot_object_recognition/RecognizeObjectAction.h"
#include "youbot_manipulation/youbot_manipulationAction.h"
#include "arm_planner/arm_planningAction.h"
#include "std_msgs/String.h"
#include "mcr_perception_msgs/ObjectList.h"
#include <boost/thread.hpp>
#include <Eigen/Dense>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

actionlib::SimpleActionServer<youbot_manipulation::youbot_manipulationAction>* as;
actionlib::SimpleActionClient<youbot_object_recognition::RecognizeObjectAction>* ac_kinect;
actionlib::SimpleActionClient<youbot_object_recognition::RecognizeObjectAction>* ac_camera_arm;
actionlib::SimpleActionClient<arm_planner::arm_planningAction>* ac_arm;

void pose2Affine(geometry_msgs::Pose pose, Eigen::Affine3d& T)
{
  Eigen::Quaterniond q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  Eigen::Matrix3d R(q);
  Eigen::Vector3d t(pose.position.x, pose.position.y, pose.position.z);
  T.linear()=R; T.translation()=t;
}

void tf2Affine(tf::StampedTransform& tf, Eigen::Affine3d& T)
{
  tf::Vector3 o=tf.getOrigin();
  tf::Quaternion q_tf=tf.getRotation();
  Eigen::Quaterniond q(q_tf[3],q_tf[0],q_tf[1],q_tf[2]);
  Eigen::Matrix3d R(q);
  Eigen::Vector3d t(o[0],o[1],o[2]);
  T.linear()=R; T.translation()=t;
}

bool computeTransformation(std::string target, std::string source, Eigen::Affine3d& T)
{
  tf::TransformListener listener;
	tf::StampedTransform transform;
	for (int i=0; i<3;i++)
	{
	  try
    {
      ros::Time now=ros::Time::now();
      listener.waitForTransform( target, source, now, ros::Duration(1));
      listener.lookupTransform( target, source, now, transform);
      //tf::transformTFToEigen(transform, T);
      tf2Affine(transform, T);
      return true;
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
    }
  }
  return false;
}

void manipulation_cb(const youbot_manipulation::youbot_manipulationGoalConstPtr & goal)
{
  youbot_manipulation::youbot_manipulationResult result;
  youbot_object_recognition::RecognizeObjectGoal kinect_goal;
  kinect_goal.object_name=goal->object_name;
  ROS_INFO("sending goal to kinect_object_recognition...");
  ac_kinect->sendGoal(kinect_goal);
  ac_kinect->waitForResult();
  youbot_object_recognition::RecognizeObjectResultConstPtr kinect_result;
  if(ac_kinect->getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
  {
		kinect_result=ac_kinect->getResult();
		std::cout<<"object_list size: "<<kinect_result->object_list.objects.size()<<std::endl;
	}
  else
  {
    ROS_INFO("object recognition failed");
    as->setAborted(result);
    return ;
  }
  Eigen::Affine3d kinect_target_pose;
  //tf::poseMsgToEigen(kinect_result->object_list.objects[0].pose.pose, kinect_target_pose);
  pose2Affine(kinect_result->object_list.objects[0].pose.pose, kinect_target_pose);
  std::cout<<"kinect pos target\n"<< kinect_target_pose.matrix()<<std::endl;
  Eigen::Quaterniond q(kinect_target_pose.linear());
  std::cout<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<std::endl;
  
  Eigen::Affine3d T_kinect_arm;
  if(!computeTransformation("arm_base", "tower_cam3d_rgb_optical_frame", T_kinect_arm))
  {
    ROS_INFO("cannot compute transformation");
    as->setAborted(result);
    return ;
  }
  Eigen::Affine3d arm_target_pose=(T_kinect_arm*kinect_target_pose);
  
  std::cout<<"target_arm\n"<<arm_target_pose.matrix()<<std::endl;
  
  arm_planner::arm_planningGoal arm_goal;
	arm_goal.mode=-1; //gaze target
	Eigen::Vector3d arm_target=arm_target_pose.translation();
	arm_goal.cartesian_position.x=arm_target(0);
	arm_goal.cartesian_position.y=arm_target(1);
	arm_goal.cartesian_position.z=arm_target(2);
	ROS_INFO("Sending Arm goal...");
	ac_arm->sendGoal(arm_goal);
	ac_arm->waitForResult();
	if(ac_arm->getState()!=actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("arm_planning failed");
    as->setAborted(result);
    return ;
  }
  sleep(2);
  
  Eigen::Affine3d T_kinect_armcamera;
  if(!computeTransformation("arm_camera_frame", "tower_cam3d_rgb_optical_frame", T_kinect_armcamera))
  {
    ROS_INFO("cannot compute transformation");
    as->setAborted(result);
    return ;
  }
  

  //Eigen::Affine3d initial_guess=(T_kinect_armcamera*kinect_target_pose);
  Eigen::Affine3d initial_guess=(T_kinect_armcamera*kinect_target_pose);
  std::cout<<"initial guess\n"<<initial_guess.matrix()<<std::endl;
  
  geometry_msgs::Pose init_guess;
  tf::poseEigenToMsg(initial_guess, init_guess);
  youbot_object_recognition::RecognizeObjectGoal arm_camera_goal;
  //arm_camera_goal.object_name=goal->object_name;
  //arm_camera_goal.object_name.data="AX-01_bearing_box";
  arm_camera_goal.object_name.data="AX-01b_bearing_box";
  //arm_camera_goal.object_name.data="AX-09_motor_with_gearbox";
  arm_camera_goal.initial_guess=init_guess;
  ROS_INFO("Sending camera arm goal...");

	ac_camera_arm->sendGoal(arm_camera_goal);
	ac_camera_arm->waitForResult();
	youbot_object_recognition::RecognizeObjectResultConstPtr camera_arm_result;
  ///camera_arm
  if(ac_camera_arm->getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
  {
		camera_arm_result=ac_camera_arm->getResult();
		Eigen::Affine3d camera_arm_target_pose;
    tf::poseMsgToEigen(camera_arm_result->object_list.objects[0].pose.pose, camera_arm_target_pose);
    
    Eigen::Affine3d T_camera_arm;
    if(!computeTransformation("arm_base", "arm_camera_frame", T_camera_arm))
    {
      ROS_INFO("cannot compute transformation");
    }
    else
    {
      arm_target_pose=(T_camera_arm*camera_arm_target_pose);
      std::cout<<"target_arm\n"<<arm_target_pose.matrix()<<std::endl;
    }
	}
	else
	  ROS_INFO("camera_arm recognition failed");
  ///
  
  arm_planner::arm_planningGoal arm_goal_grasp;
	arm_goal_grasp.mode=1; //grasp target
	arm_target=arm_target_pose.translation();
	arm_goal_grasp.cartesian_position.x=arm_target(0);
	arm_goal_grasp.cartesian_position.y=arm_target(1);
	arm_goal_grasp.cartesian_position.z=arm_target(2);
	arm_goal_grasp.gripper_roll=0;
	arm_goal_grasp.gripper_pitch=75;
	ROS_INFO("Sending Arm goal...");
	ac_arm->sendGoal(arm_goal_grasp);
	ac_arm->waitForResult();
	if(ac_arm->getState()!=actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("arm_planning failed");
    as->setAborted(result);
    return ;
  }
  
  as->setSucceeded(result);
	return ;
}

int main(int argc, char **argv){

	ros::init(argc, argv, "youbot_manipulation_server");
  	ros::NodeHandle nh;

	as=new actionlib::SimpleActionServer<youbot_manipulation::youbot_manipulationAction>(nh, "youbot_manipulation", manipulation_cb, false);
	as->start();
	
	ac_kinect = new actionlib::SimpleActionClient<youbot_object_recognition::RecognizeObjectAction> ("kinect_recognize_object", true);
	ac_camera_arm = new actionlib::SimpleActionClient<youbot_object_recognition::RecognizeObjectAction> ("camera_arm_recognize_object", true);
	ac_arm= new actionlib::SimpleActionClient<arm_planner::arm_planningAction>("arm_planner", true);
	
	ROS_INFO("waiting for kinect_recognize_object...");
  ac_kinect->waitForServer();
  ROS_INFO("waiting for camera_arm_recognize_object...");
  ac_camera_arm->waitForServer();
  ROS_INFO("waiting for arm server...");
	ac_arm->waitForServer();
  
  ROS_INFO("yuobot_manipulation is ready to compute requests");

  ros::spin();

	return 1;
}
