#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include "youbot_object_recognition/RecognizeObjectAction.h"
#include "youbot_manipulation/youbot_manipulationAction.h"
#include "arm_planner/arm_planningAction.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
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
ros::ServiceClient acquire_img_srv;

Eigen::Affine3d T_camera_arm;
Eigen::Affine3d T_kinect_armcamera;

double pitch_gaze=31;

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
      listener.waitForTransform( target, source, now, ros::Duration(.5));
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

void drop_cb(const youbot_manipulation::youbot_manipulationGoalConstPtr & goal)
{
  youbot_manipulation::youbot_manipulationResult result;
  result.move_base=false;
  
  
  Eigen::Affine3d T_kinect_arm;
  if(!computeTransformation("arm_base", "tower_cam3d_rgb_optical_frame", T_kinect_arm))
  {
    ROS_INFO("cannot compute transformation");
    as->setAborted(result);
    return ;
  }
  
  youbot_object_recognition::RecognizeObjectGoal kinect_goal;
  kinect_goal.object_name.data="all";
  ROS_INFO("sending goal to kinect_object_recognition...");
  ac_kinect->sendGoal(kinect_goal);
  ac_kinect->waitForResult();
  youbot_object_recognition::RecognizeObjectResultConstPtr kinect_result;
  if(ac_kinect->getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
  {
		kinect_result=ac_kinect->getResult();
	//	std::cout<<"object_list size: "<<kinect_result->object_list.objects.size()<<std::endl;
		if(kinect_result->object_list.objects.size()>0)
		{
		  for (int i=0; i<kinect_result->object_list.objects.size(); i++)
      {
        arm_planner::arm_planningGoal arm_goal_drop;
        Eigen::Affine3d arm_target_pose, arm_target_pose_gaze;
        Eigen::Vector3d arm_target, arm_target_gaze;
        
        Eigen::Affine3d kinect_target_pose;
        //tf::poseMsgToEigen(kinect_result->object_list.objects[0].pose.pose, kinect_target_pose);
        pose2Affine(kinect_result->object_list.objects[i].pose.pose, kinect_target_pose);
      //  std::cout<<"kinect pos target\n"<< kinect_target_pose.matrix()<<std::endl;
        
        arm_target_pose_gaze=(T_kinect_arm*kinect_target_pose);
        
      //  std::cout<<"target_arm_gaze\n"<<arm_target_pose_gaze.matrix()<<std::endl;
        
        arm_planner::arm_planningGoal arm_goal;
        arm_goal.mode=-1; //gaze target
        arm_target_gaze=arm_target_pose_gaze.translation();
        arm_goal.cartesian_position.x=arm_target_gaze(0);
        arm_goal.cartesian_position.y=arm_target_gaze(1);
        arm_goal.cartesian_position.z=arm_target_gaze(2);
        arm_goal.gripper_roll=90;
        arm_goal.gripper_pitch=pitch_gaze;
        ROS_INFO("Sending Arm goal...");
        ac_arm->sendGoal(arm_goal);
        ac_arm->waitForResult();
        if(ac_arm->getState()!=actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          ROS_INFO("arm_planning failed");
          continue;
        }
        
        sleep(1);
        std_srvs::Empty srv;
        acquire_img_srv.call(srv);
        
        if(!computeTransformation("arm_base", "arm_camera_frame", T_camera_arm))
        {
          ROS_INFO("cannot compute transformation");
          continue;
        }
        if(!computeTransformation("arm_camera_frame", "tower_cam3d_rgb_optical_frame", T_kinect_armcamera))
        {
          ROS_INFO("cannot compute transformation");
          continue;
        }

        youbot_object_recognition::RecognizeObjectGoal arm_camera_goal;
        arm_camera_goal.acquire_image.data=true;
        arm_camera_goal.object_name=goal->object_name_list[0];
        ROS_INFO("Sending camera arm goal...");

        ac_camera_arm->sendGoal(arm_camera_goal);
        ac_camera_arm->waitForResult();
        youbot_object_recognition::RecognizeObjectResultConstPtr camera_arm_result;
        ///camera_arm
        if(ac_camera_arm->getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
        {
	        camera_arm_result=ac_camera_arm->getResult();
	        if(camera_arm_result->object_list.objects[0].probability<.65) 
	        {
	          std::cout<<" score: "<<camera_arm_result->object_list.objects[0].probability<<std::endl;
	          ROS_INFO("too low score");
            continue;
	        }
	        
	        Eigen::Affine3d camera_arm_target_pose;
          tf::poseMsgToEigen(camera_arm_result->object_list.objects[0].pose.pose, camera_arm_target_pose);

          arm_target_pose=(T_camera_arm*camera_arm_target_pose);
         // std::cout<<"target_arm_final + score\n"<<arm_target_pose.matrix()<<"\n"<<camera_arm_result->object_list.objects[0].probability<<std::endl;
          
        }
        else
        {
          ROS_INFO("camera_arm recognition failed");
          continue;
        }
        
	      arm_goal_drop.mode=2; //drop target
	      arm_target=arm_target_pose.translation();
	      arm_goal_drop.cartesian_position.x=arm_target(0);
	      arm_goal_drop.cartesian_position.y=arm_target(1);
	      arm_goal_drop.cartesian_position.z=arm_target(2)+0.05;
	      arm_goal_drop.gripper_roll=0;
	      arm_goal_drop.gripper_pitch=75;
	      ROS_INFO("Sending Arm goal...");
	      ac_arm->sendGoal(arm_goal_drop);
	      ac_arm->waitForResult();
	      if(ac_arm->getState()!=actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          ROS_INFO("arm_planning failed");
          arm_planner::arm_planningResultConstPtr arm_result=ac_arm->getResult();
          if (arm_result->move_base)
          {
            result.move_base=arm_result->move_base;
            result.base_target=arm_result->base_target;
            ROS_INFO("can move the base...");
            as->setAborted(result);
            return ;
          }
          continue;
        }

        ROS_INFO("grasping succeeded");
        as->setSucceeded(result);
        return ;
      }
		}
	}
  else
  {
    ROS_INFO("object recognition failed");
    as->setAborted(result);
    return ;
  }
  
  //// backup solution /////////////
  arm_planner::arm_planningGoal arm_goal_drop;
  Eigen::Affine3d kinect_target_pose, arm_target_pose; 
  pose2Affine(kinect_result->workspace_pose, kinect_target_pose);
  arm_target_pose=(T_kinect_arm*kinect_target_pose);
  arm_goal_drop.mode=2; //drop target
  Eigen::Vector3d arm_target=arm_target_pose.translation();
  arm_goal_drop.cartesian_position.x=arm_target(0);
  arm_goal_drop.cartesian_position.y=arm_target(1);
  arm_goal_drop.cartesian_position.z=arm_target(2)+0.05;
  arm_goal_drop.gripper_roll=0;
  arm_goal_drop.gripper_pitch=75;
  ROS_INFO("Sending Arm goal...");
  ac_arm->sendGoal(arm_goal_drop);
  ac_arm->waitForResult();
  if(ac_arm->getState()!=actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("arm_planning failed");
    arm_planner::arm_planningResultConstPtr arm_result=ac_arm->getResult();
    if (arm_result->move_base)
    {
      result.move_base=arm_result->move_base;
      result.base_target=arm_result->base_target;
      ROS_INFO("can move the base...");
      as->setAborted(result);
      return ;
    }
    as->setAborted(result);
    return ;
  }
  ROS_INFO("dropping succeeded");
  as->setSucceeded(result);
  return ;
  
}

void action_cb(const youbot_manipulation::youbot_manipulationGoalConstPtr & goal)
{
  if (goal->mode==2)
  {
    drop_cb(goal);
    return;
  }

  youbot_manipulation::youbot_manipulationResult result;
  result.move_base=false;
  
  youbot_object_recognition::RecognizeObjectGoal kinect_goal;
  kinect_goal.object_name.data="all";
  ROS_INFO("sending goal to kinect_object_recognition...");
  ac_kinect->sendGoal(kinect_goal);
  ac_kinect->waitForResult();
  youbot_object_recognition::RecognizeObjectResultConstPtr kinect_result;
  if(ac_kinect->getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
  {
		kinect_result=ac_kinect->getResult();
		std::cout<<"object_list size: "<<kinect_result->object_list.objects.size()<<std::endl;
		if(kinect_result->object_list.objects.size()==0)
		{
		  ROS_INFO("object recognition failed");
      as->setAborted(result);
      return ;
		}
	}
  else
  {
    ROS_INFO("object recognition failed");
    as->setAborted(result);
    return ;
  }
  
  Eigen::Affine3d T_kinect_arm;
  if(!computeTransformation("arm_base", "tower_cam3d_rgb_optical_frame", T_kinect_arm))
  {
    ROS_INFO("cannot compute transformation");
    as->setAborted(result);
    return ;
  }
  
  for (int i=0; i<kinect_result->object_list.objects.size(); i++)
  {
    Eigen::Affine3d arm_target_pose, arm_target_pose_gaze;
    Eigen::Vector3d arm_target, arm_target_gaze;
    
    Eigen::Affine3d kinect_target_pose;
    //tf::poseMsgToEigen(kinect_result->object_list.objects[0].pose.pose, kinect_target_pose);
    pose2Affine(kinect_result->object_list.objects[i].pose.pose, kinect_target_pose);
   // std::cout<<"kinect pos target\n"<< kinect_target_pose.matrix()<<std::endl;
    
    arm_target_pose_gaze=(T_kinect_arm*kinect_target_pose);
    
    //std::cout<<"target_arm_gaze\n"<<arm_target_pose_gaze.matrix()<<std::endl;
    
    arm_planner::arm_planningGoal arm_goal;
    arm_goal.mode=-1; //gaze target
    arm_target_gaze=arm_target_pose_gaze.translation();
    arm_goal.cartesian_position.x=arm_target_gaze(0);
    arm_goal.cartesian_position.y=arm_target_gaze(1);
    arm_goal.cartesian_position.z=arm_target_gaze(2);
    arm_goal.gripper_roll=90;
    arm_goal.gripper_pitch=pitch_gaze;
    ROS_INFO("Sending Arm goal...");
    ac_arm->sendGoal(arm_goal);
    ac_arm->waitForResult();
    if(ac_arm->getState()!=actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("arm_planning failed");
      continue;
    }
    
    sleep(1);
    std_srvs::Empty srv;
    acquire_img_srv.call(srv);
    
    if(!computeTransformation("arm_base", "arm_camera_frame", T_camera_arm))
    {
      ROS_INFO("cannot compute transformation");
      continue;
    }
    if(!computeTransformation("arm_camera_frame", "tower_cam3d_rgb_optical_frame", T_kinect_armcamera))
    {
      ROS_INFO("cannot compute transformation");
      continue;
    }

    youbot_object_recognition::RecognizeObjectGoal arm_camera_goal;
    arm_camera_goal.acquire_image.data=true;
    arm_camera_goal.object_name=goal->object_name_list[0];
    ROS_INFO("Sending camera arm goal...");

    ac_camera_arm->sendGoal(arm_camera_goal);
    ac_camera_arm->waitForResult();
    youbot_object_recognition::RecognizeObjectResultConstPtr camera_arm_result;
    ///camera_arm
    if(ac_camera_arm->getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
    {
	    camera_arm_result=ac_camera_arm->getResult();
	    float good_probability=.8;
	    if(goal->object_name_list[0].data.compare("EM-01_aid_tray")==0) good_probability=.65;
	    if(camera_arm_result->object_list.objects[0].probability<good_probability) 
	    {
	      ROS_INFO("too low score");
        continue;
	    }
	    
	    Eigen::Affine3d camera_arm_target_pose;
      tf::poseMsgToEigen(camera_arm_result->object_list.objects[0].pose.pose, camera_arm_target_pose);

      arm_target_pose=(T_camera_arm*camera_arm_target_pose);
     // std::cout<<"target_arm_final + score\n"<<arm_target_pose.matrix()<<"\n"<<camera_arm_result->object_list.objects[0].probability<<std::endl;
      
    }
    else
    {
      ROS_INFO("camera_arm recognition failed");
      continue;
    }
    
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
      arm_planner::arm_planningResultConstPtr arm_result=ac_arm->getResult();
      if (arm_result->move_base)
      {
        result.move_base=arm_result->move_base;
        result.base_target=arm_result->base_target;
        ROS_INFO("can move the base...");
        as->setAborted(result);
        return ;
      }
      continue;
    }
    arm_planner::arm_planningGoal arm_goal_rest;
    arm_goal_rest.mode=-2; //grasping rest position
    ROS_INFO("Sending Arm goal...");
    ac_arm->sendGoal(arm_goal_rest);
    ac_arm->waitForResult();

    ROS_INFO("grasping succeeded");
    as->setSucceeded(result);
    return ;
  }
  
  /// backup solution (grasp considering the first kinect init guess) ///
  Eigen::Affine3d kinect_target_pose, arm_target_pose; 
  //tf::poseMsgToEigen(kinect_result->object_list.objects[0].pose.pose, kinect_target_pose);
  pose2Affine(kinect_result->object_list.objects[0].pose.pose, kinect_target_pose);
  arm_target_pose=(T_kinect_arm*kinect_target_pose);
  arm_planner::arm_planningGoal arm_goal_grasp;
  arm_goal_grasp.mode=1; //grasp target
  Eigen::Vector3d arm_target=arm_target_pose.translation();
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
    arm_planner::arm_planningResultConstPtr arm_result=ac_arm->getResult();
    if (arm_result->move_base)
    {
      result.move_base=arm_result->move_base;
      result.base_target=arm_result->base_target;
      ROS_INFO("can move the base...");
      as->setAborted(result);
      return ;
    }
    as->setAborted(result);
    return ;
  }
  arm_planner::arm_planningGoal arm_goal_rest;
  arm_goal_rest.mode=-2; //grasping rest position
  ROS_INFO("Sending Arm goal...");
  ac_arm->sendGoal(arm_goal_rest);
  ac_arm->waitForResult();

  ROS_INFO("grasping succeeded");
  as->setSucceeded(result);
  return ;
}

int main(int argc, char **argv){

	ros::init(argc, argv, "youbot_manipulation_server");
  	ros::NodeHandle nh;
  	
  acquire_img_srv = nh.serviceClient<std_srvs::Empty>("acquire_image");

	as=new actionlib::SimpleActionServer<youbot_manipulation::youbot_manipulationAction>(nh, "youbot_manipulation", action_cb, false);
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
