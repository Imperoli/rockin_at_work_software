/*
 * planning_request.cpp
 *
 *      Author: Marco Imperoli
 */

#include "arm_planner/utils.h"
#include "ros/ros.h"
#include "arm_planner/TargetRequest.h"
#include "arm_planner/Obstacle_msg.h"
#include "std_msgs/Bool.h"
#include <actionlib/client/simple_action_client.h>
#include <arm_planner/arm_planningAction.h>


bool new_request=false;
bool request_accomplished=false;
bool request_failed=false;
Eigen::Vector3d target_cartesian_position, target_cartesian_normal, target_cartesian_position_noisy, target_cartesian_normal_noisy;
std::vector<Obstacle> obstacles, obstacles_noisy;
double gripper_roll=0;
double gripper_pitch=0;


/*void response_cb(const std_msgs::Bool::ConstPtr& msg)
{
  request_accomplished=msg->data;
  if(!msg->data) request_failed=true;
  if(msg->data) std::cout<<"execution succeded!"<<std::endl;
  else std::cout<<"execution failed!"<<std::endl;

  new_request=true;
}*/

void getObstacles(){
  obstacles.clear();
  Obstacle o1(.2,.1,.2,.03);

  Obstacle o3(.2,0,0.4,.04);
  Obstacle o4(.2,0.1,0.1,.03);
  
  Obstacle o12(Eigen::Vector3d(-.015,-.265,-.1),Eigen::Vector3d(.015,-.235,0.03));

  //obstacles.push_back(o1);
  //obstacles.push_back(o2);
  //obstacles.push_back(o3);
  //obstacles.push_back(o4);
  
  //obstacles.push_back(o12);

  obstacles_noisy=obstacles;
}

void addObstaclesNoise(){
  int range=200; int offset=100;
  for(int i=0;i<obstacles.size();i++){
    double noise0=(double)(rand()%range-offset)/3000.0;
    double noise1=(double)(rand()%range-offset)/3000.0;
    double noise2=(double)(rand()%range-offset)/3000.0;
    Eigen::Vector3d noise(noise0,noise1,noise2);
    if(obstacles[i].radius<0){
      obstacles_noisy[i].max_position=obstacles[i].max_position+noise;
      obstacles_noisy[i].min_position=obstacles[i].min_position+noise;
    }else{
      obstacles_noisy[i].position=obstacles[i].position+noise;
    }
  }
}

void moveObstacles(){
  int range=200; int offset=100;
  for(int i=0;i<obstacles.size();i++){
    Eigen::Vector3d noise(.001,0,0);
    if(obstacles[i].radius<0){
      obstacles_noisy[i].max_position+=noise;
      obstacles_noisy[i].min_position+=noise;
    }else{
      obstacles_noisy[i].position+=noise;
    }
  }
}

void addTargetNoise(){
  int range=200; int offset=100;
  double noise0=(double)(rand()%range-offset)/7000.0;
  double noise1=(double)(rand()%range-offset)/7000.0;
  double noise2=(double)(rand()%range-offset)/7000.0;
  Eigen::Vector3d noise(noise0,noise1,noise2);
  target_cartesian_position_noisy=target_cartesian_position+noise;
}

void moveTarget(){
  target_cartesian_position_noisy(2)+=.0005;
}

void checkIfFinished(actionlib::SimpleActionClient<arm_planner::arm_planningAction>& ac){
  actionlib::SimpleClientGoalState state = ac.getState();
  if(state==actionlib::SimpleClientGoalState::SUCCEEDED||state==actionlib::SimpleClientGoalState::ABORTED){
    std::cout<<state.toString().c_str()<<std::endl;
    new_request=true;
  }
}

int main(int argc, char **argv){

  ros::init(argc, argv, "planning_request");
    ros::NodeHandle nh;

  ros::Publisher target_pub = nh.advertise<arm_planner::TargetRequest>("/target_request", 1);
  ros::Publisher obst_pub = nh.advertise<arm_planner::Obstacles>("/obstacles_position", 1);
  //ros::Subscriber sub = nh.subscribe("/planner_response", 1, response_cb);

  actionlib::SimpleActionClient<arm_planner::arm_planningAction> ac("arm_planner", true);
  arm_planner::arm_planningGoal goal;

  arm_planner::TargetRequest target_request;
  arm_planner::Obstacles obstacles_msg;

  getObstacles();
  ros::Rate lr(50);

  int mode=0;

  float inValue;
  std::cout << "type mode " << std::endl;
  std::cin >> mode;
  //if(mode!=4&&mode!=3){
    std::cout << "type x " << std::endl;
    std::cin >> inValue;
    target_cartesian_position(0)=(inValue);
    std::cout << "type y " << std::endl;
    std::cin >> inValue;
    target_cartesian_position(1)=(inValue);
    std::cout << "type z " << std::endl;
    std::cin >> inValue;
    target_cartesian_position(2)=(inValue);

    std::cout << "type gripper roll " << std::endl;
    std::cin >> gripper_roll;
    std::cout << "type gripper pitch " << std::endl;
    std::cin >> gripper_pitch;
  
    ac.waitForServer();
    //std::cout<<"ooooooooooooooooooooooooooo"<<std::endl;
      target_cartesian_position_noisy=target_cartesian_position;
       TargetRequest2RosGoal(target_cartesian_position_noisy, gripper_pitch, gripper_roll, mode, "rgb", goal);

        ac.sendGoal(goal);

  //}
  //target_cartesian_normal_noisy=target_cartesian_normal;
  target_cartesian_position_noisy=target_cartesian_position;
  obstacles_noisy=obstacles;

  while(nh.ok()){

    //addObstaclesNoise();
    //addTargetNoise();
    //moveObstacles();
    //moveTarget();
    TargetRequest2RosMsg(target_cartesian_position_noisy, gripper_pitch, gripper_roll, "rgb", target_request);
    target_pub.publish(target_request);
    ros::spinOnce();
    
    obstacles2RosMsg(obstacles_noisy, obstacles_msg);
    obst_pub.publish(obstacles_msg);
    ros::spinOnce();
    
    lr.sleep();
    checkIfFinished(ac);

    if(new_request){
      new_request=false;
      sleep(.2);
      std::cout << "type mode " << std::endl;
      std::cin >> mode;
      //if(mode!=4&&mode!=3){
        std::cout << "type x " << std::endl;
        std::cin >> inValue;
        target_cartesian_position(0)=(inValue);
        std::cout << "type y " << std::endl;
        std::cin >> inValue;
        target_cartesian_position(1)=(inValue);
        std::cout << "type z " << std::endl;
        std::cin >> inValue;
        target_cartesian_position(2)=(inValue);

        std::cout << "type gripper roll " << std::endl;
        std::cin >> gripper_roll;
        std::cout << "type gripper pitch " << std::endl;
        std::cin >> gripper_pitch;
        ac.waitForServer();
    
      target_cartesian_position_noisy=target_cartesian_position;
       TargetRequest2RosGoal(target_cartesian_position_noisy, gripper_pitch, gripper_roll, mode, "rgb",goal);

        ac.sendGoal(goal);
      //}
      
      target_cartesian_position_noisy=target_cartesian_position;
    }
    
  }

  return 1;
}


