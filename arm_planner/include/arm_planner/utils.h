/*
 * utils.h
 *
 *      Author: Marco Imperoli
 */

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <Eigen/Dense>
#include "arm_planner/control_point.h"
#include "arm_planner/obstacle.h"
#include "arm_planner/PlanningSceneFrame.h"
#include "arm_planner/ControlPoint_msg.h"
#include "arm_planner/Obstacles.h"
#include "arm_planner/TargetRequest.h"
//#include "arm_planner/PlanningRequest.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
//#include "arm_planner/ArmMotionPlanning.h"
#include <arm_planner/arm_planningAction.h>
//#include "arm_planner/ArmMotionPlanning.h"

#include <iostream>
#include <fstream>


#define l2 0.155
#define l3 0.135
//#define l4 0.2175
#define l4 0.3025 //0.3125 // new gripper
 
#define lox 0.033
#define loz 0.1472
#define lax 0

#define baseLink_armLink0_offset_x 0.143
#define baseLink_armLink0_offset_z 0.046

using namespace arm_planner;
using namespace std;
using namespace Eigen;

const double joint_offsets[5] = {2.9496, 1.1344, -2.5481, 1.7889, 2.9234};

inline void forwardKin(Eigen::Matrix<double,5,1>& j_pos, arm_planner::ControlPoint& c_point){
  double lax_=lax; double lox_=lox; double loz_=loz; double l2_=l2; double l3_=l3; double l4_=l4;
  if(c_point.link==1){
    loz_=c_point.d;
    c_point.position(0) = lax_ ;
    c_point.position(1) = 0;
    c_point.position(2) = loz_ ;
  }
  if(c_point.link==2){
    l2_=c_point.d;
    c_point.position(0) = lax_ + cos(j_pos(0)) * (lox_ + l2_ * sin(j_pos(1)));
    c_point.position(1) = sin(j_pos(0)) * (lox_ + l2_ * sin(j_pos(1)));
    c_point.position(2) = loz_ + l2_ * cos(j_pos(1));
  }
  if(c_point.link==3){
    l3_=c_point.d;
    c_point.position(0) = lax_ + cos(j_pos(0)) * (lox_ + l3_ * sin(j_pos(1) + j_pos(2)) + l2_ * sin(j_pos(1)));
    c_point.position(1) = sin(j_pos(0)) * (lox_ + l3_ * sin(j_pos(1) + j_pos(2)) + l2_ * sin(j_pos(1)));
    c_point.position(2) = loz_ + l3_ * cos(j_pos(1) + j_pos(2)) + l2_ * cos(j_pos(1));
  }
  if(c_point.link==4){
    l4_=c_point.d;
    c_point.position(0) = lax_ + cos(j_pos(0)) * (lox_ + l3_ * sin(j_pos(1) + j_pos(2)) + l2_ * sin(j_pos(1)) + l4_ * sin(j_pos(1) + j_pos(2) + j_pos(3)));
    c_point.position(1) = sin(j_pos(0)) * (lox_ + l3_ * sin(j_pos(1) + j_pos(2)) + l2_ * sin(j_pos(1)) + l4_ * sin(j_pos(1) + j_pos(2) + j_pos(3)));
    c_point.position(2) = loz_ + l3_ * cos(j_pos(1) + j_pos(2)) + l2_ * cos(j_pos(1)) + l4_ * cos(j_pos(1) + j_pos(2) + j_pos(3));
  }
}

inline void forwardKinEE(Eigen::Matrix<double,5,1>& j_pos, Eigen::Vector3d& ee_pos){
  double lax_=lax; double lox_=lox; double loz_=loz; double l2_=l2; double l3_=l3; double l4_=l4;

    ee_pos(0) = lax_ + cos(j_pos(0)) * (lox_ + l3_ * sin(j_pos(1) + j_pos(2)) + l2_ * sin(j_pos(1)) + l4_ * sin(j_pos(1) + j_pos(2) + j_pos(3)));
    ee_pos(1) = sin(j_pos(0)) * (lox_ + l3_ * sin(j_pos(1) + j_pos(2)) + l2_ * sin(j_pos(1)) + l4_ * sin(j_pos(1) + j_pos(2) + j_pos(3)));
    ee_pos(2) = loz_ + l3_ * cos(j_pos(1) + j_pos(2)) + l2_ * cos(j_pos(1)) + l4_ * cos(j_pos(1) + j_pos(2) + j_pos(3));

}

inline void getTwists(Eigen::MatrixXd & xi_1, Eigen::MatrixXd & xi_2, Eigen::MatrixXd & xi_3, Eigen::MatrixXd & xi_4,
                      Eigen::MatrixXd & xi_5, Eigen::MatrixXd & tf, Eigen::Matrix<double,5,1>& pos)
{
  double cos1 = cos(pos(0));
  double sin1 = sin(pos(0));
  double sin2 = sin(pos(1));
  double cos2 = cos(pos(1));
  double sin3 = sin(pos(2));
  double cos3 = cos(pos(2));
  double sin4 = sin(pos(3));
  double cos4 = cos(pos(3));
  double cos5 = cos(pos(4));
  double sin5 = sin(pos(4));

  xi_1(0, 0) = cos1;
  xi_1(0, 1) = -sin1;
  xi_1(0, 2) = 0;
  xi_1(0, 3) = -lax * (cos1 - 1.0);
  xi_1(1, 0) = sin1;
  xi_1(1, 1) = cos1;
  xi_1(1, 2) = 0;
  xi_1(1, 3) = -lax * sin1;
  xi_1(2, 0) = 0;
  xi_1(2, 1) = 0;
  xi_1(2, 2) = 1.0;
  xi_1(2, 3) = 0;
  xi_1(3, 0) = 0;
  xi_1(3, 1) = 0;
  xi_1(3, 2) = 0;
  xi_1(3, 3) = 1.0;

  xi_2(0, 0) = cos2;
  xi_2(0, 1) = 0;
  xi_2(0, 2) = sin2;
  xi_2(0, 3) = -loz * sin2 - (cos2 - 1.0) * (lax + lox);
  xi_2(1, 0) = 0;
  xi_2(1, 1) = 1.0;
  xi_2(1, 2) = 0;
  xi_2(1, 3) = 0;
  xi_2(2, 0) = -sin2;
  xi_2(2, 1) = 0;
  xi_2(2, 2) = cos2;
  xi_2(2, 3) = sin2 * (lax + lox) - loz * (cos2 - 1.0);
  xi_2(3, 0) = 0;
  xi_2(3, 1) = 0;
  xi_2(3, 2) = 0;
  xi_2(3, 3) = 1.0;

  xi_3(0, 0) = cos3;
  xi_3(0, 1) = 0;
  xi_3(0, 2) = sin3;
  xi_3(0, 3) = -sin3 * (l2 + loz) - (cos3 - 1.0) * (lax + lox);
  xi_3(1, 0) = 0;
  xi_3(1, 1) = 1.0;
  xi_3(1, 2) = 0;
  xi_3(1, 3) = 0;
  xi_3(2, 0) = -sin3;
  xi_3(2, 1) = 0;
  xi_3(2, 2) = cos3;
  xi_3(2, 3) = sin3 * (lax + lox) - (cos3 - 1.0) * (l2 + loz);
  xi_3(3, 0) = 0;
  xi_3(3, 1) = 0;
  xi_3(3, 2) = 0;
  xi_3(3, 3) = 1.0;

  xi_4(0, 0) = cos4;
  xi_4(0, 1) = 0;
  xi_4(0, 2) = sin4;
  xi_4(0, 3) = -(cos4 - 1.0) * (lax + lox) - sin4 * (l2 + l3 + loz);
  xi_4(1, 0) = 0;
  xi_4(1, 1) = 1.0;
  xi_4(1, 2) = 0;
  xi_4(1, 3) = 0;
  xi_4(2, 0) = -sin4;
  xi_4(2, 1) = 0;
  xi_4(2, 2) = cos4;
  xi_4(2, 3) = sin4 * (lax + lox) - (cos4 - 1.0) * (l2 + l3 + loz);
  xi_4(3, 0) = 0;
  xi_4(3, 1) = 0;
  xi_4(3, 2) = 0;
  xi_4(3, 3) = 1.0;

  xi_5(0, 0) = cos5;
  xi_5(0, 1) = -sin5;
  xi_5(0, 2) = 0;
  xi_5(0, 3) = -(cos5 - 1.0) * (lax + lox);
  xi_5(1, 0) = sin5;
  xi_5(1, 1) = cos5;
  xi_5(1, 2) = 0;
  xi_5(1, 3) = -sin5 * (lax + lox);
  xi_5(2, 0) = 0;
  xi_5(2, 1) = 0;
  xi_5(2, 2) = 1.0;
  xi_5(2, 3) = 0;
  xi_5(3, 0) = 0;
  xi_5(3, 1) = 0;
  xi_5(3, 2) = 0;
  xi_5(3, 3) = 1.0;

  tf(0, 0) = 0;
  tf(0, 1) = 0;
  tf(0, 2) = -1.0;
  tf(0, 3) = lax + lox;
  tf(1, 0) = 0;
  tf(1, 1) = 1.0;
  tf(1, 2) = 0;
  tf(1, 3) = 0;
  tf(2, 0) = 1.0;
  tf(2, 1) = 0;
  tf(2, 2) = 0;
  tf(2, 3) = l2 + l3 + l4 + loz;
  tf(3, 0) = 0;
  tf(3, 1) = 0;
  tf(3, 2) = 0;
  tf(3, 3) = 1.0;
}

inline void forwardKin(Eigen::MatrixXd & xi_1, Eigen::MatrixXd & xi_2, Eigen::MatrixXd & xi_3, Eigen::MatrixXd & xi_4,
                       Eigen::MatrixXd & xi_5, Eigen::MatrixXd & tf, Eigen::Affine3d & cart_pos)
{
  Eigen::MatrixXd temp(4, 4);
  temp = xi_1 * xi_2 * xi_3 * xi_4 * xi_5 * tf;
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      cart_pos(i, j) = temp(i, j);
    }
  }
}

inline void forwardKinOrientationEE(Eigen::Matrix<double,5,1>& j_pos, Eigen::Matrix3d& ee_orient){
  Eigen::Affine3d  cart_pos;
  MatrixXd xi_1(4, 4);
   MatrixXd xi_2(4, 4);
   MatrixXd xi_3(4, 4);
   MatrixXd xi_4(4, 4);
   MatrixXd xi_5(4, 4);
   MatrixXd tf(4, 4);
   getTwists(xi_1, xi_2, xi_3, xi_4, xi_5, tf, j_pos);
   forwardKin(xi_1, xi_2, xi_3, xi_4, xi_5, tf, cart_pos);
   ee_orient=cart_pos.rotation();
}

inline void forwardKinematicsEE(Eigen::Matrix<double,5,1>& j_pos, Eigen::Affine3d&  cart_pos){
  MatrixXd xi_1(4, 4);
  MatrixXd xi_2(4, 4);
  MatrixXd xi_3(4, 4);
  MatrixXd xi_4(4, 4);
  MatrixXd xi_5(4, 4);
  MatrixXd tf(4, 4);
  getTwists(xi_1, xi_2, xi_3, xi_4, xi_5, tf, j_pos);
  forwardKin(xi_1, xi_2, xi_3, xi_4, xi_5, tf, cart_pos); 
}

inline Eigen::VectorXd youbot2matlab(const Eigen::VectorXd j_pos)
{
  Eigen::VectorXd pos(5);
  pos(0)=-(j_pos(0)-joint_offsets[0]);
  pos(1)=j_pos(1)-joint_offsets[1];
  pos(2)=j_pos(2)-joint_offsets[2];
  pos(3)=j_pos(3)-joint_offsets[3];
  pos(4)=-(j_pos(4)-joint_offsets[4]);
  return pos;
}

inline Eigen::VectorXd matlab2youbot(const Eigen::VectorXd j_pos)
{
  Eigen::VectorXd pos(5);
  pos(0)=-j_pos(0)+joint_offsets[0];
  pos(1)=+j_pos(1)+joint_offsets[1];
  pos(2)=+j_pos(2)+joint_offsets[2];
  pos(3)=+j_pos(3)+joint_offsets[3];
  pos(4)=-j_pos(4)+joint_offsets[4];
  return pos;
}


inline void compute_ee_position(Eigen::Matrix<double, 5, 1>& j_pos, Eigen::Vector3d& ee_pos){
  Eigen::Matrix<double, 5,1> j_pos2;

  //j_pos(0)=joints_pos[4]; j_pos(1)=joints_pos[3]; j_pos(2)=joints_pos[2]; j_pos(3)=joints_pos[1]; j_pos(4)=joints_pos[0];
  j_pos2=youbot2matlab(j_pos);
  forwardKinEE(j_pos2, ee_pos);

}

inline void compute_ee_orientation(Eigen::Matrix<double, 5, 1>& j_pos, Eigen::Matrix3d& ee_orient){
  Eigen::Matrix<double, 5,1> j_pos2;

  //j_pos(0)=joints_pos[4]; j_pos(1)=joints_pos[3]; j_pos(2)=joints_pos[2]; j_pos(3)=joints_pos[1]; j_pos(4)=joints_pos[0];
  j_pos2=youbot2matlab(j_pos);
  forwardKinOrientationEE(j_pos2, ee_orient);
}

inline void compute_ee_pose(Eigen::Matrix<double, 5, 1>& j_pos, Eigen::Matrix4d& ee_T){
  ee_T.setIdentity();
  Eigen::Matrix<double, 5,1> j_pos2;

  //j_pos(0)=joints_pos[4]; j_pos(1)=joints_pos[3]; j_pos(2)=joints_pos[2]; j_pos(3)=joints_pos[1]; j_pos(4)=joints_pos[0];
  j_pos2=youbot2matlab(j_pos);
  Eigen::Vector3d ee_pos;
  forwardKinEE(j_pos2, ee_pos);
  Eigen::Matrix3d ee_orient;
  forwardKinOrientationEE(j_pos2, ee_orient);
  ee_T.block<3,3>(0,0)=ee_orient;
  ee_T.col(3).head(3)=ee_pos;
}

inline void compute_ee_pose(Eigen::Matrix<double, 5, 1>& j_pos, Eigen::Affine3d& ee_T){

  Eigen::Matrix<double, 5,1> j_pos2;

  //j_pos(0)=joints_pos[4]; j_pos(1)=joints_pos[3]; j_pos(2)=joints_pos[2]; j_pos(3)=joints_pos[1]; j_pos(4)=joints_pos[0];
  j_pos2=youbot2matlab(j_pos);
  Eigen::Vector3d ee_pos;
  forwardKinEE(j_pos2, ee_pos);
  Eigen::Matrix3d ee_orient;
  forwardKinOrientationEE(j_pos2, ee_orient);
  ee_T.linear()=ee_orient;
  ee_T.translation()=ee_pos;
}

inline void smoothRot(Eigen::Matrix3d & out)
{
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      out(i, j) = round((double)out(i, j) * 10000.0) / 10000.0;
    }
  }
}


inline double yawFromTarget(Eigen::Vector3d& target){
  if(fabs(target(0))<.00005&&target(1)>0) return 1.57079632679;
  else if(fabs(target(0))<.00005&&target(1)<0) return -1.57079632679;
  return atan2(target(1),target(0));
}

inline double camPitchFromTarget(Eigen::Vector3d& target, Eigen::Vector3d& j4_pos){
  Eigen::Vector3d p(target(0), target(1), j4_pos(2));
  return atan2(p(2)-target(2), (p-j4_pos).norm());
}

inline void rotToRPY(Eigen::Matrix3d & rot, Eigen::Vector3d & rpy)
{
  rpy(1) = atan2(-(double)rot(2, 0), sqrt(pow((double)rot(0, 0), 2)) + pow((double)rot(1, 0), 2));
  if (fabs((double)rpy(1) - M_PI / 2) < DBL_EPSILON)
  {
    rpy(2) = 0;
    rpy(0) = atan2((double)rot(0, 1), (double)rot(1, 1));
  }
  else if (fabs((double)rpy(1) + M_PI / 2) < DBL_EPSILON)
  {
    rpy(2) = 0;
    rpy(0) = -atan2((double)rot(0, 1), (double)rot(1, 1));
  }
  else
  {
    rpy(2) = atan2((double)rot(1, 0) / cos((double)rpy(1)), (double)rot(0, 0) / cos((double)rpy(1)));
    rpy(0) = atan2((double)rot(2, 1) / cos((double)rpy(1)), (double)rot(2, 2) / cos((double)rpy(1)));
  }
}

inline void rotToRPY2(Eigen::Matrix3d & rot, Eigen::Vector3d & rpy)
{
  Eigen::Matrix<double,3,1> euler = rot.eulerAngles(2, 1, 0);
rpy(2) = euler(0,0);
rpy(1) = euler(1,0);
rpy(0) = euler(2,0);
}

inline void gripperRollToRotMatrix(double roll, Matrix3d& rot_m){
  rot_m = AngleAxisd(roll*M_PI/180.0, Vector3d::UnitZ())
    * AngleAxisd(0.5*M_PI, Vector3d::UnitY())
    * AngleAxisd(0*M_PI, Vector3d::UnitX());
  smoothRot(rot_m);  
}

inline void rotMatrixToNorm(Matrix3d& rot_m, Eigen::Vector3d& norm){
  Eigen::Vector3d rpy;
  rotToRPY(rot_m,rpy);
  double roll=rpy(0);
  double pitch=rpy(1);
  double yaw=rpy(2);
  norm(0) = cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll);
  norm(1) = sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll);
  norm(2) = cos(pitch) * sin(roll);
  
}

inline void rpyToNorm(Eigen::Vector3d& rpy, Eigen::Vector3d& norm){
  double roll=rpy(0);
  double pitch=rpy(1);
  double yaw=rpy(2);
  norm(0) = cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll);
  norm(1) = sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll);
  norm(2) = cos(pitch) * sin(roll);
  //std::cout<<rpy<<"\n"<<std::endl;
  //std::cout<<norm<<"\n"<<std::endl;
  
}

inline void rosGoal2TargetRequest(const arm_planner::arm_planningGoalConstPtr & goal, Eigen::Matrix4d& kinect_T, Eigen::Matrix4d& rgb_T, Eigen::Matrix<double, 5, 1>& j_pos,  Eigen::Vector3d& cart_pos, Eigen::Vector3d& cart_norm, double& gripper_pitch, double& gripper_roll, int& mode){
  Eigen::Matrix4d ee_T;
  compute_ee_pose(j_pos,ee_T);
  Eigen::Vector4d t, new_t;
  /*if(goal->frame_id=="kinect"){
    t(0)=goal->cartesian_position.x; t(1)=goal->cartesian_position.y; t(2)=goal->cartesian_position.z; t(3)=1;
    Eigen::Matrix4d T=kinect_T;
    new_t=T*t;
    new_t=new_t/new_t(3);
    cart_pos=new_t.head(3);
  }else if(goal->frame_id=="rgb"){
    t(0)=goal->cartesian_position.x; t(1)=goal->cartesian_position.y; t(2)=goal->cartesian_position.z; t(3)=1;
    Eigen::Matrix4d T=ee_T*rgb_T;
    new_t=T*t;
    new_t=new_t/new_t(3);
    cart_pos=new_t.head(3);
  }*/
  //std::cout<<"target:\n"<<cart_pos<<std::endl;
  cart_pos(0)=goal->cartesian_position.x; cart_pos(1)=goal->cartesian_position.y; cart_pos(2)=goal->cartesian_position.z;
  //cart_norm(0)=goal->cartesian_normal.x; cart_norm(1)=goal->cartesian_normal.y; cart_norm(2)=goal->cartesian_normal.z;
  mode=goal->mode;
  gripper_roll=goal->gripper_roll;
  gripper_pitch=goal->gripper_pitch;
  //if(gripper_roll<-360){
  //  cart_norm=Eigen::Vector3d(0,0,0);
  //}else{

    Eigen::Vector3d rpy(gripper_roll*M_PI/180.0, gripper_pitch*M_PI/180, yawFromTarget(cart_pos));
    rpyToNorm(rpy,cart_norm);
  //}
}

inline void TargetRequest2RosGoal(Eigen::Vector3d& cart_pos, double gripper_pitch, double gripper_roll, int mode,  std::string frame_id, arm_planner::arm_planningGoal& goal){
  goal.cartesian_position.x=cart_pos(0); goal.cartesian_position.y=cart_pos(1); goal.cartesian_position.z=cart_pos(2);  
  //msg.cartesian_normal.x=cart_norm(0); msg.cartesian_normal.y=cart_norm(1); msg.cartesian_normal.z=cart_norm(2);
  goal.mode=mode;
  goal.frame_id=frame_id;
  goal.gripper_roll=gripper_roll;
  goal.gripper_pitch=gripper_pitch;
  if(goal.gripper_roll<1&&goal.gripper_roll>-1) goal.gripper_roll=1;
  if(goal.gripper_pitch<1&&goal.gripper_pitch>-1) goal.gripper_pitch=1;

}


inline void TargetRequest2RosMsg(Eigen::Vector3d& cart_pos, double gripper_pitch, double gripper_roll, std::string frame_id, arm_planner::TargetRequest& msg){
  msg.cartesian_position.x=cart_pos(0); msg.cartesian_position.y=cart_pos(1); msg.cartesian_position.z=cart_pos(2);  
  msg.gripper_roll=gripper_roll;
  msg.gripper_pitch=gripper_pitch;
  msg.frame_id=frame_id;
  if(msg.gripper_roll<1&&msg.gripper_roll>-1) msg.gripper_roll=1;
  if(msg.gripper_pitch<1&&msg.gripper_pitch>-1) msg.gripper_pitch=1;
}

inline void rosMsg2TargetRequest(arm_planner::TargetRequest& msg,Eigen::Matrix4d& kinect_T, Eigen::Matrix4d& rgb_T, Eigen::Matrix<double, 5, 1>& j_pos, Eigen::Vector3d& cart_pos, Eigen::Vector3d& cart_norm, double& gripper_pitch, double& gripper_roll){
  Eigen::Matrix4d ee_T;
  compute_ee_pose(j_pos,ee_T);
  Eigen::Vector4d t, new_t;
  /*if(msg.frame_id=="kinect"){
    t(0)=msg.cartesian_position.x; t(1)=msg.cartesian_position.y; t(2)=msg.cartesian_position.z; t(3)=1;
    Eigen::Matrix4d T=kinect_T;
    new_t=T*t;
    new_t=new_t/new_t(3);
    cart_pos=new_t.head(3);
  }else if(msg.frame_id=="rgb"){
    t(0)=msg.cartesian_position.x; t(1)=msg.cartesian_position.y; t(2)=msg.cartesian_position.z; t(3)=1;
    Eigen::Matrix4d T=ee_T*rgb_T;
    new_t=T*t;
    new_t=new_t/new_t(3);
    cart_pos=new_t.head(3);
  }*/
  cart_pos(0)=msg.cartesian_position.x; cart_pos(1)=msg.cartesian_position.y; cart_pos(2)=msg.cartesian_position.z;
  gripper_roll=msg.gripper_roll;
  gripper_pitch=msg.gripper_pitch;
  //if(gripper_roll<-360){
  //  cart_norm=Eigen::Vector3d(0,0,0);
  //}else{

    Eigen::Vector3d rpy(gripper_roll*M_PI/180.0, gripper_pitch*M_PI/180, yawFromTarget(cart_pos));
    rpyToNorm(rpy,cart_norm);
  //}
}

inline void obstacles2RosMsg(std::vector<Obstacle>& ob, arm_planner::Obstacles& msg){
  msg.obstacles.clear();
  arm_planner::Obstacle_msg ob_msg;
  for(int i=0;i<ob.size();i++){
    ob_msg.position.x=ob[i].position(0); ob_msg.position.y=ob[i].position(1); ob_msg.position.z=ob[i].position(2);
    ob_msg.min_position.x=ob[i].min_position(0); ob_msg.min_position.y=ob[i].min_position(1); ob_msg.min_position.z=ob[i].min_position(2);
    ob_msg.max_position.x=ob[i].max_position(0); ob_msg.max_position.y=ob[i].max_position(1); ob_msg.max_position.z=ob[i].max_position(2);
    ob_msg.radius=ob[i].radius;
    msg.obstacles.push_back(ob_msg);
  }
}

inline void rosMsg2Obstacles(arm_planner::Obstacles& msg, Eigen::Matrix4d& kinect_T, std::vector<Obstacle>& obs){
  obs.clear();
  Obstacle ob;
  Eigen::Vector4d t, new_t;
  for(int i=0; i<msg.obstacles.size();i++){
    ob.position(0)=msg.obstacles[i].position.x; ob.position(1)=msg.obstacles[i].position.y; ob.position(2)=msg.obstacles[i].position.z;
    ob.min_position(0)=msg.obstacles[i].min_position.x; ob.min_position(1)=msg.obstacles[i].min_position.y; ob.min_position(2)=msg.obstacles[i].min_position.z;
    ob.max_position(0)=msg.obstacles[i].max_position.x; ob.max_position(1)=msg.obstacles[i].max_position.y; ob.max_position(2)=msg.obstacles[i].max_position.z;
    ob.radius=msg.obstacles[i].radius;
    
    Eigen::Matrix4d T=kinect_T;
    t.head(3)=ob.position; t(3)=1;
    new_t=T*t;
    new_t=new_t/new_t(3);
    ob.position=new_t.head(3);
    t.head(3)=ob.min_position; t(3)=1;
    new_t=T*t;
    new_t=new_t/new_t(3);
    ob.min_position=new_t.head(3);
    t.head(3)=ob.max_position; t(3)=1;
    new_t=T*t;
    new_t=new_t/new_t(3);
    ob.max_position=new_t.head(3);
    
    for (int k=0; k<3; k++){
      if(ob.min_position(k)>ob.max_position(k)){
        double temp=ob.min_position(k);
        ob.min_position(k)=ob.max_position(k);
        ob.max_position(k)=temp;
      }
    }
    
    obs.push_back(ob);
  }
}

inline void planningSceneFrame2RosMsg(std::vector<ControlPoint>& cp2,std::vector<ControlPoint>& cp, std::vector<Obstacle>& ob, std::vector<Eigen::Vector3d>& ee_traj, Eigen::Vector3d& cart_pos, std::vector<Eigen::Vector3d>& man_area, Eigen::Matrix3d& ee_orient, arm_planner::PlanningSceneFrame& msg){
  msg.control_points.clear();
  msg.real_control_points.clear();
  msg.obstacles.clear();
  msg.ee_traj.clear();
  msg.manipulation_area.clear();
  msg.ee_orient.clear();
  arm_planner::Obstacle_msg ob_msg;
  arm_planner::ControlPoint_msg cp_msg;
  geometry_msgs::Point p;
  for(int i=0; i<cp.size();i++){
    cp_msg.position.x=cp[i].position(0); cp_msg.position.y=cp[i].position(1); cp_msg.position.z=cp[i].position(2);
    cp_msg.radius=cp[i].radius; cp_msg.link=cp[i].link; cp_msg.d=cp[i].d;
    msg.control_points.push_back(cp_msg);
  }
  for(int i=0; i<cp2.size();i++){
    cp_msg.position.x=cp2[i].position(0); cp_msg.position.y=cp2[i].position(1); cp_msg.position.z=cp2[i].position(2);
    cp_msg.radius=cp2[i].radius; cp_msg.link=cp2[i].link; cp_msg.d=cp2[i].d;
    msg.real_control_points.push_back(cp_msg);
  }
  for(int i=0;i<ob.size();i++){
    ob_msg.position.x=ob[i].position(0); ob_msg.position.y=ob[i].position(1); ob_msg.position.z=ob[i].position(2);
    ob_msg.min_position.x=ob[i].min_position(0); ob_msg.min_position.y=ob[i].min_position(1); ob_msg.min_position.z=ob[i].min_position(2);
    ob_msg.max_position.x=ob[i].max_position(0); ob_msg.max_position.y=ob[i].max_position(1); ob_msg.max_position.z=ob[i].max_position(2);
    ob_msg.radius=ob[i].radius;
    msg.obstacles.push_back(ob_msg);
  }
  for(int i=0; i<ee_traj.size(); i++){
    p.x=ee_traj[i](0); p.y=ee_traj[i](1); p.z=ee_traj[i](2);
    msg.ee_traj.push_back(p);
  }
  for(int i=0; i<man_area.size(); i++){
    p.x=man_area[i](0); p.y=man_area[i](1); p.z=man_area[i](2);
    msg.manipulation_area.push_back(p);
  }
  msg.cartesian_position.x=cart_pos(0); msg.cartesian_position.y=cart_pos(1); msg.cartesian_position.z=cart_pos(2);
  
  for(int i=0;i<3;i++){
    for(int j=0; j<3; j++){
      msg.ee_orient.push_back(ee_orient(i,j));
    }
  }
}

inline void rosMsg2PlanningSceneFrame(arm_planner::PlanningSceneFrame& msg, std::vector<ControlPoint>& cps2,std::vector<ControlPoint>& cps, std::vector<Obstacle>& obs, std::vector<Eigen::Vector3d>& ee_traj, Eigen::Vector3d& cart_pos, std::vector<Eigen::Vector3d>& man_area, Eigen::Matrix3d& ee_orient){
  cps.clear();
  cps2.clear();
  obs.clear();
  ee_traj.clear();
  man_area.clear();
  Obstacle ob;
  ControlPoint cp;
  Eigen::Vector3d p;
  for(int i=0; i<msg.control_points.size();i++){
    cp.position(0)=msg.control_points[i].position.x; cp.position(1)=msg.control_points[i].position.y; cp.position(2)=msg.control_points[i].position.z;
    cp.radius=msg.control_points[i].radius; cp.link=msg.control_points[i].link; cp.d=msg.control_points[i].d;
    cps.push_back(cp);
  }
  for(int i=0; i<msg.real_control_points.size();i++){
    cp.position(0)=msg.real_control_points[i].position.x; cp.position(1)=msg.real_control_points[i].position.y; cp.position(2)=msg.real_control_points[i].position.z;
    cp.radius=msg.real_control_points[i].radius; cp.link=msg.real_control_points[i].link; cp.d=msg.real_control_points[i].d;
    cps2.push_back(cp);
  }
  for(int i=0; i<msg.obstacles.size();i++){
    ob.position(0)=msg.obstacles[i].position.x; ob.position(1)=msg.obstacles[i].position.y; ob.position(2)=msg.obstacles[i].position.z;
    ob.min_position(0)=msg.obstacles[i].min_position.x; ob.min_position(1)=msg.obstacles[i].min_position.y; ob.min_position(2)=msg.obstacles[i].min_position.z;
    ob.max_position(0)=msg.obstacles[i].max_position.x; ob.max_position(1)=msg.obstacles[i].max_position.y; ob.max_position(2)=msg.obstacles[i].max_position.z;
    ob.radius=msg.obstacles[i].radius;
    obs.push_back(ob);
  }
  for(int i=0; i<msg.ee_traj.size(); i++){
    p(0)=msg.ee_traj[i].x; p(1)=msg.ee_traj[i].y; p(2)=msg.ee_traj[i].z; 
    ee_traj.push_back(p);
  }
  for(int i=0; i<msg.manipulation_area.size(); i++){
    p(0)=msg.manipulation_area[i].x; p(1)=msg.manipulation_area[i].y; p(2)=msg.manipulation_area[i].z; 
    man_area.push_back(p);
  }
  cart_pos(0)=msg.cartesian_position.x; cart_pos(1)=msg.cartesian_position.y; cart_pos(2)=msg.cartesian_position.z;
  
  int k=0;
  for(int i=0;i<9;i+=3){
    ee_orient(k,0)=msg.ee_orient[i]; ee_orient(k,1)=msg.ee_orient[i+1]; ee_orient(k,2)=msg.ee_orient[i+2];
    k++;
  }
}


inline void compute_control_points_pos(std::vector<ControlPoint>& control_points, std::vector<double>& joints_pos){
  Eigen::Matrix<double, 5,1> j_pos;
  j_pos(0)=joints_pos[4]; j_pos(1)=joints_pos[3]; j_pos(2)=joints_pos[2]; j_pos(3)=joints_pos[1]; j_pos(4)=joints_pos[0];
  j_pos=youbot2matlab(j_pos);

  for(int i=0;i<control_points.size();i++){
    if(i==12){
      forwardKin(j_pos, control_points[i+3]);
      Eigen::Matrix3d ee_orient; forwardKinOrientationEE(j_pos, ee_orient);
      control_points[i].position=ee_orient*Eigen::Vector3d(-.155,-.07,0)+control_points[i+3].position;
      control_points[i+1].position=ee_orient*Eigen::Vector3d(-.105,-.07,0)+control_points[i+3].position;
      control_points[i+2].position=ee_orient*Eigen::Vector3d(-.205,-.07,0)+control_points[i+3].position;
      break;
    }
    if (control_points[i].link>0){
      forwardKin(j_pos, control_points[i]);
    }
  }
}

inline void compute_control_points_pos(std::vector<ControlPoint>& control_points, Eigen::Matrix<double, 5,1>& joints_pos){
  Eigen::Matrix<double, 5,1> j_pos;
  //j_pos(0)=joints_pos[4]; j_pos(1)=joints_pos[3]; j_pos(2)=joints_pos[2]; j_pos(3)=joints_pos[1]; j_pos(4)=joints_pos[0];
  j_pos=youbot2matlab(joints_pos);

  for(int i=0;i<control_points.size();i++){
    if(i==12){
      forwardKin(j_pos, control_points[i+3]);
      Eigen::Matrix3d ee_orient; forwardKinOrientationEE(j_pos, ee_orient);
      control_points[i].position=ee_orient*Eigen::Vector3d(-.155,-.07,0)+control_points[i+3].position;
      control_points[i+1].position=ee_orient*Eigen::Vector3d(-.105,-.07,0)+control_points[i+3].position;
      control_points[i+2].position=ee_orient*Eigen::Vector3d(-.205,-.07,0)+control_points[i+3].position;
      break;
    }
    if (control_points[i].link>0){
      forwardKin(j_pos, control_points[i]);
    }
  }
}

inline void compute_control_points_pos_without_offsets(std::vector<ControlPoint>& control_points, Eigen::Matrix<double, 5,1>& j_pos){
  //Eigen::Matrix<double, 5,1> j_pos;
  //j_pos(0)=joints_pos[4]; j_pos(1)=joints_pos[3]; j_pos(2)=joints_pos[2]; j_pos(3)=joints_pos[1]; j_pos(4)=joints_pos[0];
  //j_pos=youbot2matlab(joints_pos);

  for(int i=0;i<control_points.size();i++){
    if(i==12){
      forwardKin(j_pos, control_points[i+3]);
      Eigen::Matrix3d ee_orient; forwardKinOrientationEE(j_pos, ee_orient);
      control_points[i].position=ee_orient*Eigen::Vector3d(-.155,-.07,0)+control_points[i+3].position;
      control_points[i+1].position=ee_orient*Eigen::Vector3d(-.105,-.07,0)+control_points[i+3].position;
      control_points[i+2].position=ee_orient*Eigen::Vector3d(-.205,-.07,0)+control_points[i+3].position;
      break;
    }
    if (control_points[i].link>0){
      forwardKin(j_pos, control_points[i]);
    }
  }
}

inline bool check_obstacles_collision(std::vector<ControlPoint>& control_points, std::vector<Obstacle>& obstacles, Eigen::Matrix<double, 5,1>& joints_pos){
  compute_control_points_pos(control_points,joints_pos);

  // Obstacles Collisions
  for(int i=0;i<control_points.size();i++){
    for(int j=0;j<obstacles.size();j++){
      if(control_points[i].link>0){
        if(obstacles[j].normal(0)!=0||obstacles[j].normal(1)!=0||obstacles[j].normal(2)!=0){//// plane obst
          Eigen::Vector3d diff=control_points[i].position-obstacles[j].position;
          if(diff.dot(obstacles[j].normal)<control_points[i].radius) return true;
        }else if(obstacles[j].radius>=0){//// sphere obst
          if((control_points[i].position-obstacles[j].position).norm()<(control_points[i].radius+obstacles[j].radius)) return true;
        }else{///box obst
          if((control_points[i].position(0)+control_points[i].radius)>obstacles[j].min_position(0)&&(control_points[i].position(1)+control_points[i].radius)>obstacles[j].min_position(1)&&(control_points[i].position(2)+control_points[i].radius)>obstacles[j].min_position(2)&&(control_points[i].position(0)-control_points[i].radius)<obstacles[j].max_position(0)&&(control_points[i].position(1)-control_points[i].radius)<obstacles[j].max_position(1)&&(control_points[i].position(2)-control_points[i].radius)<obstacles[j].max_position(2))
            return true;
        }
      }
    }
  }

  return false;
}

inline bool check_obstacles_collision(std::vector<ControlPoint>& control_points, std::vector<Obstacle>& obstacles, std::vector<double>& joints_pos){
  compute_control_points_pos(control_points,joints_pos);

  // Obstacles Collisions
  for(int i=0;i<control_points.size();i++){
    for(int j=0;j<obstacles.size();j++){
      if(control_points[i].link>0){
        if(obstacles[j].normal(0)!=0||obstacles[j].normal(1)!=0||obstacles[j].normal(2)!=0){//// plane obst
          Eigen::Vector3d diff=control_points[i].position-obstacles[j].position;
          if(diff.dot(obstacles[j].normal)<control_points[i].radius) return true;
        }else if(obstacles[j].radius>=0){//// sphere obst
          if((control_points[i].position-obstacles[j].position).norm()<(control_points[i].radius+obstacles[j].radius)) return true;
        }else{///box obst
          if((control_points[i].position(0)+control_points[i].radius)>obstacles[j].min_position(0)&&(control_points[i].position(1)+control_points[i].radius)>obstacles[j].min_position(1)&&(control_points[i].position(2)+control_points[i].radius)>obstacles[j].min_position(2)&&(control_points[i].position(0)-control_points[i].radius)<obstacles[j].max_position(0)&&(control_points[i].position(1)-control_points[i].radius)<obstacles[j].max_position(1)&&(control_points[i].position(2)-control_points[i].radius)<obstacles[j].max_position(2))
            return true;
        }
      }
    }
  }

  return false;
}

inline void grow_control_points(std::vector<ControlPoint>& control_points, double d, std::vector<ControlPoint>& control_points_big){
  control_points_big=control_points;
  for(int i=0;i<control_points.size();i++){
    control_points_big[i].radius+=d;
  }
}

inline bool compute_base_target(std::vector<Eigen::Vector3d>& man_area, Eigen::Vector3d& b_t){
  int range=man_area.size();
  if(range>0){
    double b_t_norm=10000;
    for(int i=0;i<10;i++){
      int index=rand()%range;
      if(man_area[index].norm()<b_t_norm){
        b_t=man_area[index];
        b_t_norm=b_t.norm();
      }
    }
    //b_t(0)-=baseLink_armLink0_offset_x;
    return true;
  }
  return false;
}

inline bool compute_manipulation_area(std::vector<std::vector<Eigen::Vector3d> >& work_space, Eigen::Vector3d& target, double gripper_pitch, double angle_in_radians, std::vector<Eigen::Vector3d>& manipulation_area){
  ros::Time tic=ros::Time::now();
  manipulation_area.clear();
  double eps=.019;

  int index_ws=0;
  if(gripper_pitch<0||gripper_pitch>90) return false;
  else{
    if(gripper_pitch<15) index_ws=0;
    else{
      if(gripper_pitch<37.5) index_ws=1;
      else{
        if(gripper_pitch<52.5) index_ws=2;
        else{
          if(gripper_pitch<67.5) index_ws=3;
          else index_ws=4;
        }
      }
    }
  }  

  int index=0;  
  while(index<work_space[index_ws].size()&&fabs(work_space[index_ws][index](2)-target(2))>eps){
    index++;
  }
  if (index==work_space[index_ws].size()) return false;

  Eigen::Vector3d p(0,0,0);
  double flag=work_space[index_ws][index](2);
  double theta=angle_in_radians;
  while(flag==work_space[index_ws][index](2)&&index<work_space[index_ws].size()){
    //if(work_space[index_ws][index](0)>0 && work_space[index_ws][index](1)<=1.5*work_space[index_ws][index](0) && work_space[index_ws][index](1)>=-1.5*work_space[index_ws][index](0)){
    if(work_space[index_ws][index](1)<0){
      p(0)=target(0)-(work_space[index_ws][index](0)*cos(theta)+work_space[index_ws][index](1)*sin(theta));
      p(1)=target(1)-(-work_space[index_ws][index](0)*sin(theta)+work_space[index_ws][index](1)*cos(theta));
      //p(0)=target(0)-work_space[index](0);
      //p(1)=target(1)-work_space[index](1);
      manipulation_area.push_back(p);
    }
    index+=1;
  }
  /*ros::Time toc=ros::Time::now();
  double elapsed_secs = (toc-tic).toSec();
  cout<<"elapsed secs man_area "<<elapsed_secs<<endl;
*/
  return true;
}

inline void loadSingleWorkSpace(std::string filename, std::vector<Eigen::Vector3d>& work_space){
  std::fstream file(filename.c_str(), std::ios_base::in);

  double x,y,z;
  while (file >> x)
  {
    file >> y; file >> z;
    Eigen::Vector3d work_spacep(x,y,z);
    work_space.push_back(work_spacep);
  }
  std::cout<<"size work_space "<<work_space.size()<<std::endl;
  file.close();
}

inline void loadWorkSpace(std::string dir, std::vector<std::vector<Eigen::Vector3d> >& workspace){
  std::string file_name;
  workspace.clear();
  std::vector<Eigen::Vector3d> ws;
  file_name=dir+"ws_0.txt";
  loadSingleWorkSpace(file_name, ws);
  workspace.push_back(ws);
  ws.clear();
  file_name=dir+"ws_30.txt";
  loadSingleWorkSpace(file_name, ws);
  workspace.push_back(ws);
  ws.clear();
  file_name=dir+"ws_45.txt";
  loadSingleWorkSpace(file_name, ws);
  workspace.push_back(ws);
  ws.clear();
  file_name=dir+"ws_60.txt";
  loadSingleWorkSpace(file_name, ws);
  workspace.push_back(ws);
  ws.clear();
  file_name=dir+"ws_75.txt";
  loadSingleWorkSpace(file_name, ws);
  workspace.push_back(ws);
}

inline void loadMatrix(std::string dir, Eigen::Matrix4d& kinect_T, Eigen::Matrix4d& rgb_T){
  std::string file_name ,file_name2;
  file_name=dir+"kinect.txt";
  file_name2=dir+"rgb.txt";
  std::fstream file(file_name.c_str(), std::ios_base::in);
  std::fstream file2(file_name2.c_str(), std::ios_base::in);
  double x;
  for (int i=0;i<4;i++){
    for(int j=0; j<4; j++){
      file>>kinect_T(i,j);
      file2>>rgb_T(i,j);
    }
  }
  file.close(); file2.close();
  std::cout<<"M\n"<<kinect_T<<std::endl;
  std::cout<<"M2\n"<<rgb_T<<std::endl;
}


inline void flip_trajectory(std::vector<trajectory_msgs::JointTrajectoryPoint>& points){
  std::vector<trajectory_msgs::JointTrajectoryPoint> temp;
  for(int i=points.size()-1;i>=0;i--){
    temp.push_back(points[i]);
  }
  points=temp;
}



#endif
