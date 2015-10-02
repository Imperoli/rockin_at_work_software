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
#include "next_best_view.h"

#include <fstream>

actionlib::SimpleActionServer<youbot_manipulation::youbot_manipulationAction>* as;
actionlib::SimpleActionClient<youbot_object_recognition::RecognizeObjectAction>* ac_kinect;
actionlib::SimpleActionClient<youbot_object_recognition::RecognizeObjectAction>* ac_camera_arm;
actionlib::SimpleActionClient<arm_planner::arm_planningAction>* ac_arm;
ros::ServiceClient acquire_img_srv;
ros::ServiceClient ik_client;

Eigen::Affine3d T_camera_arm;

double pitch_gaze=25;


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
      listener.waitForTransform( target, source, now, ros::Duration(.25));
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

void action_cb(const youbot_manipulation::youbot_manipulationGoalConstPtr & goal)
{
  youbot_manipulation::youbot_manipulationResult result;
  result.move_base=false;
  
  /*youbot_object_recognition::RecognizeObjectGoal kinect_goal;
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
  
  Eigen::Affine3d kinect_target_pose;
  //tf::poseMsgToEigen(kinect_result->object_list.objects[0].pose.pose, kinect_target_pose);
  pose2Affine(kinect_result->object_list.objects[0].pose.pose, kinect_target_pose);
  
  Eigen::Affine3d initial_guess_pos;
  initial_guess_pos=(T_kinect_arm*kinect_target_pose);
  Eigen::Affine3d arm_target_pose=initial_guess_pos;*/
  
  Eigen::Affine3d initial_guess_pos;
  Eigen::Matrix4d m;
  m<<-0.0648834,  -0.412018,   0.908863, -0.20495,
    -0.00499045,  -0.910637,  -0.413178,  -0.455945,
    0.99788,  -0.031344,   0.057029, -0.0319087,  
    0,          0,          0,          1;
  initial_guess_pos.matrix()=m;
  Eigen::Affine3d arm_target_pose=initial_guess_pos;
  
  std::cout<<"init_guess\n"<< initial_guess_pos.matrix()<<std::endl;
  
  
  std::vector<GazeView> gaze_views;
  extractNViews(3, ik_client, initial_guess_pos, gaze_views);
 //ractNRandomViews(3, ik_client, initial_guess_pos, gaze_views);
  
  std::vector<Eigen::Affine3d> gaze_views_poses;

  for(int i=0; i<gaze_views.size(); i++)
  {
    arm_planner::arm_planningGoal arm_goal;
    /*arm_goal.mode=-1; //gaze target
    Eigen::Vector3d arm_target_gaze=gaze_views[i].target;
    arm_goal.cartesian_position.x=arm_target_gaze(0);
    arm_goal.cartesian_position.y=arm_target_gaze(1);
    arm_goal.cartesian_position.z=arm_target_gaze(2);
    arm_goal.gripper_roll=90;
    arm_goal.gripper_pitch=gaze_views[i].pitch;
    arm_goal.gaze_distance=gaze_views[i].distance;*/
    arm_goal.mode=10; //gaze target
    for (int h=0;h<5 ;h++)
      arm_goal.joints_target[h]=gaze_views[i].joints_target(h);
      
    ROS_INFO("Sending Arm goal...");
    ac_arm->sendGoal(arm_goal);
    ac_arm->waitForResult();
    if(ac_arm->getState()!=actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("arm_planning failed");
      as->setAborted(result);
      return;
    }
    
    sleep(.5);
    std_srvs::Empty srv;
    acquire_img_srv.call(srv);
    
    if(!computeTransformation("arm_base", "arm_camera_frame", T_camera_arm))
    {
      ROS_INFO("cannot compute transformation");
      as->setAborted(result);
      return;
    }
    
    Eigen::Matrix4d calib; calib<< 0.999275,  -0.0363594,    0.011269, 0.000974965,
                                 0.035974,    0.998818,   0.0326987,  -0.0104986,
                                 -0.0124446,  -0.0322696,    0.999402,  -0.0105937,
                                 0,0,0,1;
    
    Eigen::Affine3d view, T_offset;
    T_offset=Eigen::Affine3d::Identity();
    view=T_camera_arm*T_offset;
    gaze_views_poses.push_back(view);
    
    /*Eigen::Affine3d T_camera_ee, T_ee_arm;
    computeTransformation("end_effector_edge", "arm_camera_frame", T_camera_ee);
    computeTransformation("arm_base", "end_effector_edge", T_ee_arm);
    //std::stringstream ss; ss<<"info_"<<i<<".txt";
    //std::ofstream of(ss.str().c_str());of <<"view:\n"<<view.matrix()<<"\n\nT_ee_arm:\n"<<T_ee_arm.matrix()<<"\n\nT_camera_ee:"<<T_camera_ee.matrix()<<std::endl;*/
    std::stringstream ss; ss<<"/home/spqr/obj_rec_dataset/multi/info_"<<i<<".txt";
    std::ofstream of(ss.str().c_str());of <<"camera pose in arm_base frame:\n"<<view.matrix()<<std::endl;

  }

  youbot_object_recognition::RecognizeObjectGoal arm_camera_goal;
  std::vector<geometry_msgs::Pose> views;
  for(int i=0; i<gaze_views_poses.size(); i++){
    geometry_msgs::Pose view;
    tf::poseEigenToMsg(gaze_views_poses[i], view);
    views.push_back(view);
  }
  gaze_views_poses.clear();
  
 /* arm_camera_goal.views=views;
  arm_camera_goal.object_name=goal->object_name_list[0];
  ROS_INFO("Sending camera arm goal...");
  ac_camera_arm->sendGoal(arm_camera_goal);
  ac_camera_arm->waitForResult();
  youbot_object_recognition::RecognizeObjectResultConstPtr camera_arm_result;
  ///camera_arm
  if(ac_camera_arm->getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    camera_arm_result=ac_camera_arm->getResult();
    if(camera_arm_result->object_list.objects[0].probability<.8) 
    {
      ROS_INFO("too low score");
      as->setAborted(result);
      return ;
    }
    
    Eigen::Affine3d camera_arm_target_pose;
    tf::poseMsgToEigen(camera_arm_result->object_list.objects[0].pose.pose, camera_arm_target_pose);

    //arm_target_pose=(T_camera_arm*camera_arm_target_pose);
    arm_target_pose=camera_arm_target_pose;
    std::cout<<"target_arm_final + score\n"<<arm_target_pose.matrix()<<"\n"<<camera_arm_result->object_list.objects[0].probability<<std::endl;
    
  }
  else
  {
    ROS_INFO("camera_arm recognition failed");
    as->setAborted(result);
    return ;
  }*/
  
  arm_planner::arm_planningGoal arm_goal_grasp;
  arm_goal_grasp.mode=1; //grasp target
  arm_goal_grasp.cartesian_position.x=-.18;
  arm_goal_grasp.cartesian_position.y=-.45;
  arm_goal_grasp.cartesian_position.z=-.055;
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
 
  /*arm_planner::arm_planningGoal arm_goal_rest;
  arm_goal_rest.mode=-2; //grasping rest position
  ROS_INFO("Sending Arm goal...");
  ac_arm->sendGoal(arm_goal_rest);
  ac_arm->waitForResult();*/
  
  arm_goal_grasp.mode=2; //grasp target
  arm_goal_grasp.cartesian_position.x=.35;
  arm_goal_grasp.cartesian_position.y=.01;
  arm_goal_grasp.cartesian_position.z=.0;
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
  








  arm_goal_grasp.mode=1; //grasp target
  arm_goal_grasp.cartesian_position.x=-.27;
  arm_goal_grasp.cartesian_position.y=-.43;
  arm_goal_grasp.cartesian_position.z=-.06;
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
 
  /*arm_planner::arm_planningGoal arm_goal_rest;
  arm_goal_rest.mode=-2; //grasping rest position
  ROS_INFO("Sending Arm goal...");
  ac_arm->sendGoal(arm_goal_rest);
  ac_arm->waitForResult();*/
  
  arm_goal_grasp.mode=2; //grasp target
  arm_goal_grasp.cartesian_position.x=.35;
  arm_goal_grasp.cartesian_position.y=.2;
  arm_goal_grasp.cartesian_position.z=-.03;
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

  ROS_INFO("grasping succeeded");
  as->setSucceeded(result);
  return ;
}

int main(int argc, char **argv){

	ros::init(argc, argv, "youbot_manipulation_server");
  	ros::NodeHandle nh;
  	
  acquire_img_srv = nh.serviceClient<std_srvs::Empty>("acquire_image");
  ik_client= nh.serviceClient<arm_planner::check_IK_feasibility>("arm_planner/check_IK_feasibility");

	as=new actionlib::SimpleActionServer<youbot_manipulation::youbot_manipulationAction>(nh, "youbot_manipulation", action_cb, false);
	as->start();
	

	ac_arm= new actionlib::SimpleActionClient<arm_planner::arm_planningAction>("arm_planner", true);
	

  ROS_INFO("waiting for arm server...");
	ac_arm->waitForServer();
  
  ROS_INFO("yuobot_manipulation is ready to compute requests");

  ros::spin();

	return 1;
}
