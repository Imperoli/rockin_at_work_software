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
#include "next_best_view_new.h"
#include "youbot_object_recognition/volume_in_camera_fov.h"

#include <fstream>

actionlib::SimpleActionServer<youbot_manipulation::youbot_manipulationAction>* as;
actionlib::SimpleActionClient<youbot_object_recognition::RecognizeObjectAction>* ac_kinect;
actionlib::SimpleActionClient<youbot_object_recognition::RecognizeObjectAction>* ac_detection;
actionlib::SimpleActionClient<youbot_object_recognition::RecognizeObjectAction>* ac_localization;
actionlib::SimpleActionClient<youbot_object_recognition::RecognizeObjectAction>* ac_nbv;
actionlib::SimpleActionClient<youbot_object_recognition::RecognizeObjectAction>* ac_nbv_MI;
actionlib::SimpleActionClient<arm_planner::arm_planningAction>* ac_arm;
ros::ServiceClient acquire_img_srv, get_poses_volume_in_fov;
ros::ServiceClient ik_client, ee_pose_client;

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

bool move_arm_to(View& view)
{
  youbot_manipulation::youbot_manipulationResult result;
  arm_planner::arm_planningGoal arm_goal;
  arm_goal.mode=10; //gaze target
  for (int h=0;h<5 ;h++)
    arm_goal.joints_target[h]=view.joints_target(h);
    
  ROS_INFO("Sending Arm goal...");
  ac_arm->sendGoal(arm_goal);
  ac_arm->waitForResult();
  if(ac_arm->getState()!=actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("arm_planning failed");
    as->setAborted(result);
    return false;
  }
  
  if(!computeTransformation("arm_base", "arm_camera_frame", view.camera_pose))
  {
    ROS_INFO("cannot compute transformation");
    as->setAborted(result);
    return false;
  }
  return true;
}

void acquire_image()
{
  sleep(2);
  std_srvs::Empty srv;
  acquire_img_srv.call(srv);
}

void detectObject(const View& cam_view, std::vector<ObjectCandidate>& candidates)
{
  candidates.clear();
  youbot_object_recognition::RecognizeObjectGoal detection_goal;
  ac_detection->sendGoal(detection_goal);
  ac_detection->waitForResult();
  youbot_object_recognition::RecognizeObjectResultConstPtr ris;
  ris=ac_detection->getResult();
  for(int i=0; i<ris->object_guess_poses.size(); i++){
    Eigen::Affine3d pose;
    tf::poseMsgToEigen (ris->object_guess_poses[i], pose);
    pose=cam_view.camera_pose*pose;
    ObjectCandidate oc; oc.pose=pose; oc.avgDist=ris->object_guess_avgDist[i];
    candidates.push_back(oc);
    std::cout<<oc.pose.matrix()<<"\n"<<oc.avgDist<<std::endl;
  }
}

bool localizeObject(const std::vector<View>& cam_views, const std::vector<ObjectCandidate>& candidates, Eigen::Affine3d& object_pose)
{
  youbot_object_recognition::RecognizeObjectGoal localization_goal;
  for(int i=0; i<cam_views.size(); i++){
    geometry_msgs::Pose p;
    tf::poseEigenToMsg(cam_views[i].camera_pose,p);
    localization_goal.views.push_back(p);
  }
  for(int i=0; i<candidates.size(); i++){
    geometry_msgs::Pose p;
    tf::poseEigenToMsg(candidates[i].pose,p);
    localization_goal.initial_guess_list.push_back(p);
  }
  std::cout<<"sending localization goal..."<<std::endl;
  ac_localization->sendGoal(localization_goal);
  ac_localization->waitForResult();
  youbot_object_recognition::RecognizeObjectResultConstPtr ris;
  ris=ac_localization->getResult();
  if(ac_localization->getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    tf::poseMsgToEigen (ris->object_pose, object_pose);  
    std::cout<<"final localization result:\n"<<object_pose.matrix()<<"\n"<<std::endl;
    return true;
  }
  else
  {
    std::cout<<"localization failed"<<std::endl;
    return false;
  }
}

void compute_volume_views(const std::vector<View>& fov_views, const std::vector<ObjectCandidate>& candidates, std::vector<View>& volume_views)
{
  volume_views.clear();
  youbot_object_recognition::volume_in_camera_fov srv;
  get_poses_volume_in_fov.call(srv);
  for(int i=0; i<srv.response.filtered_views_indices.size(); i++){
    volume_views.push_back(fov_views[srv.response.filtered_views_indices[i]]);
  } 
  std::cout<<"volume_views: "<<volume_views.size()<<std::endl;
} 

void computeNextBestView(const std::vector<View>& good_views, const std::vector<View>& prev_views, const std::vector<ObjectCandidate>& candidates, View& nbv)
{
  youbot_object_recognition::RecognizeObjectGoal nbv_goal;
  for(int i=0; i<candidates.size(); i++){
    geometry_msgs::Pose p;
    tf::poseEigenToMsg(candidates[i].pose,p);
    nbv_goal.initial_guess_list.push_back(p);
    nbv_goal.object_guess_avgDist.push_back(candidates[i].avgDist);
  }
  for(int i=0; i<good_views.size(); i++){
    geometry_msgs::Pose p;
    tf::poseEigenToMsg(good_views[i].camera_pose,p);
    nbv_goal.views.push_back(p);
  }
  for(int i=0; i<prev_views.size(); i++){
    geometry_msgs::Pose p;
    tf::poseEigenToMsg(prev_views[i].camera_pose,p);
    nbv_goal.previous_views.push_back(p);
  }
  std::cout<<"sending next best view goal..."<<std::endl;
  ac_nbv->sendGoal(nbv_goal);
  ac_nbv->waitForResult();
  youbot_object_recognition::RecognizeObjectResultConstPtr ris;
  ris=ac_nbv->getResult();
  nbv=good_views[ris->next_best_view_index];
  
  std::cout<<"next_best_view result:\n"<<nbv.camera_pose.matrix()<<"\n"<<std::endl;
}

void computeNBestViews(int N, const std::vector<View>& fov_views, const View& first_view, const std::vector<ObjectCandidate>& candidates, std::vector<View>& next_views, std::vector<View>& prev_views)
{
  next_views.clear();
  prev_views.clear();
  std::vector<View> good_views;
  prev_views.push_back(first_view);
  for(int i=0; i<N; i++){
    computeNGoodViews(50,fov_views, prev_views, good_views);
    View nbv;
    computeNextBestView(good_views, prev_views, candidates, nbv);
    prev_views.push_back(nbv);
    next_views.push_back(nbv);
  }
}

void computeNextBestViewMI(const std::vector<View>& good_views, const std::vector<View>& prev_views, const View& current_view, const std::vector<ObjectCandidate>& candidates, View& nbv)
{
  youbot_object_recognition::RecognizeObjectGoal nbv_goal;
  for(int i=0; i<candidates.size(); i++){
    geometry_msgs::Pose p;
    tf::poseEigenToMsg(candidates[i].pose,p);
    nbv_goal.initial_guess_list.push_back(p);
    nbv_goal.object_guess_avgDist.push_back(candidates[i].avgDist);
  }
  for(int i=0; i<good_views.size(); i++){
    geometry_msgs::Pose p;
    tf::poseEigenToMsg(good_views[i].camera_pose,p);
    nbv_goal.views.push_back(p);
  }
  for(int i=0; i<prev_views.size(); i++){
    geometry_msgs::Pose p;
    tf::poseEigenToMsg(prev_views[i].camera_pose,p);
    nbv_goal.previous_views.push_back(p);
  }
  geometry_msgs::Pose current;
  tf::poseEigenToMsg(current_view.camera_pose,current);
  nbv_goal.current_view=current;
  
  std::cout<<"sending next best view goal..."<<std::endl;
  ac_nbv_MI->sendGoal(nbv_goal);
  ac_nbv_MI->waitForResult();
  youbot_object_recognition::RecognizeObjectResultConstPtr ris;
  ris=ac_nbv_MI->getResult();
  nbv=good_views[ris->next_best_view_index];
  
  std::cout<<"next_best_view result:\n"<<nbv.camera_pose.matrix()<<"\n"<<std::endl;
}

void computeNBestViewsMI(int N, const std::vector<View>& fov_views, const View& first_view, const std::vector<ObjectCandidate>& candidates, std::vector<View>& next_views, std::vector<View>& prev_views)
{
  next_views.clear();
  prev_views.clear();
  std::vector<View> good_views;
  prev_views.push_back(first_view);
  View current_view=first_view;
  for(int i=0; i<N; i++){
    computeNGoodViews(50,fov_views, prev_views, good_views);
    View nbv;
    computeNextBestViewMI(good_views, prev_views, current_view, candidates, nbv);
    prev_views.push_back(nbv);
    next_views.push_back(nbv);
    
    if(!move_arm_to(nbv)) return;
    acquire_image();
    current_view=nbv;
  }
}

void compute_experiments()
{
  int nbv_ok=0;
  int nbv_mi_ok=0;
  int geometric_nbv_ok=0;
  int random_ok=0;
  int total=0;
  
  Eigen::Affine3d initial_guess_pos, object_pose;
  Eigen::Matrix4d m;
  m<<-0.0648834,  -0.412018,   0.908863, -0.000495,
    -0.00499045,  -0.910637,  -0.413178,  -0.390075945,
    0.99788,  -0.031344,   0.057029, -0.07519087,  
    0,          0,          0,          1;
  initial_guess_pos.matrix()=m;
  Eigen::Affine3d arm_target_pose=initial_guess_pos;
  
  for(int exp=0; exp<100; exp++)
  {  
    std::vector<View> next_views, random_views, fov_views, volume_views, previous_views;
    std::vector<ObjectCandidate> candidates; 

    computeFOVViews(initial_guess_pos, fov_views);
    
    extractNRandomViews(3,fov_views,random_views);
    if(!move_arm_to(random_views[0])) return;
    acquire_image();
   
    detectObject(random_views[0], candidates);
    
    compute_volume_views(fov_views, candidates, volume_views);
    
    /////NBV MUTUAL INFORMATION STRATEGY//////////////
    computeNBestViewsMI(2, volume_views, random_views[0], candidates, next_views, previous_views); 
    if (localizeObject(previous_views, candidates, object_pose)) nbv_mi_ok++;

    /////NBV DISOCCLUSION STRATEGY//////////////
    //computeNBestViews(2, fov_views, random_views[0], candidates, next_views, previous_views); 
    computeNBestViews(2, volume_views, random_views[0], candidates, next_views, previous_views); 
    for(int i=0; i<next_views.size(); i++){
      if(!move_arm_to(next_views[i])) return;
      acquire_image();
    }
    if (localizeObject(previous_views, candidates, object_pose)) nbv_ok++;
    
    /////GEOMETRIC NBV STRATEGY////////////
    computeGeometricNBV(volume_views,  previous_views);
    for(int i=0; i<previous_views.size(); i++){
      if(!move_arm_to(previous_views[i])) return;
      acquire_image();
    }
    if (localizeObject(previous_views, candidates, object_pose)) geometric_nbv_ok++;
    
    ////RANDOM STRATEGY/////////
    for(int i=0; i<random_views.size(); i++){
      if(!move_arm_to(random_views[i])) return;
      acquire_image();
    }
    if (localizeObject(random_views, candidates, object_pose)) random_ok++;
    
    total++;
    std::cout<<"# exp: "<<exp+1<<" nbv_mi_ok: "<<nbv_mi_ok<<" nbv_ok: "<<nbv_ok<<" random_ok: "<<random_ok<<std::endl;
  }
  
  ROS_INFO("experiments succeeded");
  std::cout<<" nbv_mi_ok: "<<nbv_mi_ok<<" nbv_ok: "<<nbv_ok<<" random_ok: "<<random_ok<<" total: "<<total<<std::endl;
  return ;
}

int main(int argc, char **argv){

	ros::init(argc, argv, "youbot_manipulation_server");
  	ros::NodeHandle nh;
  	
  	srand (time(NULL));
  	
  acquire_img_srv = nh.serviceClient<std_srvs::Empty>("acquire_image");
  get_poses_volume_in_fov=nh.serviceClient<youbot_object_recognition::volume_in_camera_fov>("compute_camera_poses_fit_the_volume");
  ik_client= nh.serviceClient<arm_planner::check_IK_feasibility>("arm_planner/check_IK_feasibility");
  ee_pose_client= nh.serviceClient<arm_planner::get_ee_pose>("arm_planner/get_ee_pose");
	
	//ac_kinect = new actionlib::SimpleActionClient<youbot_object_recognition::RecognizeObjectAction> ("kinect_recognize_object", true);
	//ac_camera_arm = new actionlib::SimpleActionClient<youbot_object_recognition::RecognizeObjectAction> ("camera_arm_recognize_object", true);
	ac_nbv_MI = new actionlib::SimpleActionClient<youbot_object_recognition::RecognizeObjectAction> ("compute_next_best_view_mutual_information", true);
	ac_nbv = new actionlib::SimpleActionClient<youbot_object_recognition::RecognizeObjectAction> ("compute_next_best_view_disocclusion", true);
	ac_localization = new actionlib::SimpleActionClient<youbot_object_recognition::RecognizeObjectAction> ("localize_object", true);
	ac_detection = new actionlib::SimpleActionClient<youbot_object_recognition::RecognizeObjectAction> ("detect_object", true);
	ac_arm= new actionlib::SimpleActionClient<arm_planner::arm_planningAction>("arm_planner", true);
	
	
/*	ROS_INFO("waiting for kinect_recognize_object...");
  ac_kinect->waitForServer();*/
  ROS_INFO("waiting for next_best_view_server...");
  ac_nbv->waitForServer();
  ac_nbv_MI->waitForServer();
  ROS_INFO("waiting for localize_object...");
  ac_localization->waitForServer();
  ROS_INFO("waiting for detect_object...");
  ac_detection->waitForServer();
  ROS_INFO("waiting for arm server...");
	ac_arm->waitForServer();
  
  ROS_INFO("yuobot_manipulation_nbv is ready to compute experiments");
  compute_experiments();
  //ros::spin();

	return 1;
}
