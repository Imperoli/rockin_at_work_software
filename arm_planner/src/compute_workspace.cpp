/*
 * compute_workspace.cpp
 *
 *      Author: Marco Imperoli
 */


#include "arm_planner/planner.h"
#include <iostream>
#include <fstream>

using namespace arm_planner;
using namespace std;

Planner* planner;

bool compute_approach_target(Eigen::Vector3d& target, Eigen::Vector3d& pos_target, double gripper_pitch, double gripper_roll){
  trajectory_msgs::JointTrajectory straight_traj_null;
  double a=.07; double yaw=yawFromTarget(target);
  double pitch=gripper_pitch*M_PI/180.0;
  Eigen::Vector3d displacement(-cos(pitch)*cos(yaw), -cos(pitch)*sin(yaw), sin(pitch));
  pos_target=target+(a*displacement);
  planner->max_acc=0.4;
  planner->max_vel=0.04;
  if(planner->plan_straight_traj(pos_target, target, gripper_pitch, gripper_roll, straight_traj_null)){
    return true;
  }

  pos_target=Eigen::Vector3d(0,0,0);
  return false;
}

int main(int argc, char **argv){

  ros::init(argc, argv, "compute_workspace");
    ros::NodeHandle nh;
  planner=new Planner(nh);

  ofstream file;
  file.open("/home/marco/catkin_ws/src/rockin/arm_planner/src/ws_high.txt");

  int count=0;
  
  double gripper_pitch=75;
  double gripper_roll=-90;
  Eigen::Vector3d v;


  ros::Time tic=ros::Time::now();
  for(double z=-.11;z<.5;z+=.02){
    for(double x=-.5; x<.75; x+=.02){
      std::cout<<"z: "<<z<<" x: "<<x<<std::endl;
      for(double y=-.75; y<0.01;y+=.02){

        /// not above base ///
        if((x<planner->fixed_obstacles[0].max_position(0)&&x>planner->fixed_obstacles[0].min_position(0)&&y<planner->fixed_obstacles[0].max_position(1)&&y>planner->fixed_obstacles[0].min_position(1))||(x<planner->fixed_obstacles[6].max_position(0)&&x>planner->fixed_obstacles[6].min_position(0)&&y<planner->fixed_obstacles[6].max_position(1)&&y>planner->fixed_obstacles[6].min_position(1))||
(x<planner->fixed_obstacles[2].position(0)+planner->fixed_obstacles[2].radius&&x>planner->fixed_obstacles[2].position(0)-planner->fixed_obstacles[2].radius&&y<planner->fixed_obstacles[2].position(1)+planner->fixed_obstacles[2].radius&&y>planner->fixed_obstacles[2].position(1)-planner->fixed_obstacles[2].radius)||
(x<planner->fixed_obstacles[3].position(0)+planner->fixed_obstacles[3].radius&&x>planner->fixed_obstacles[3].position(0)-planner->fixed_obstacles[3].radius&&y<planner->fixed_obstacles[3].position(1)+planner->fixed_obstacles[3].radius&&y>planner->fixed_obstacles[3].position(1)-planner->fixed_obstacles[3].radius)||
(x<planner->fixed_obstacles[4].position(0)+planner->fixed_obstacles[4].radius&&x>planner->fixed_obstacles[4].position(0)-planner->fixed_obstacles[4].radius&&y<planner->fixed_obstacles[4].position(1)+planner->fixed_obstacles[4].radius&&y>planner->fixed_obstacles[4].position(1)-planner->fixed_obstacles[4].radius)||
(x<planner->fixed_obstacles[5].position(0)+planner->fixed_obstacles[5].radius&&x>planner->fixed_obstacles[5].position(0)-planner->fixed_obstacles[5].radius&&y<planner->fixed_obstacles[5].position(1)+planner->fixed_obstacles[5].radius&&y>planner->fixed_obstacles[5].position(1)-planner->fixed_obstacles[5].radius)) 
          continue;
        ///////////////////

        Eigen::Vector3d target(x,y,z);
        if(compute_approach_target(target, v, gripper_pitch, gripper_roll)){
          file<<x<<" "<<y<<" "<<z<<std::endl;
          if(y<-.001)file<<x<<" "<<-y<<" "<<z<<std::endl;
          count++;
        }
      }
    }
  }
  ros::Time toc=ros::Time::now();
  double elapsed_secs = (toc-tic).toSec();
  cout<<"elapsed secs "<<elapsed_secs<<endl;
  //file<<"elapsed secs "<<elapsed_secs<<endl;
  file.close();

  cout<<"tot "<<count<<endl;
  
  return 1;
}
