#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "youbot_manipulation/youbot_manipulationAction.h"

actionlib::SimpleActionClient<youbot_manipulation::youbot_manipulationAction>* ac;

int main(int argc, char **argv){
	
	ros::init(argc, argv, "youbot_manipulation_client");
  ros::NodeHandle nh;
	
	ac = new actionlib::SimpleActionClient<youbot_manipulation::youbot_manipulationAction> ("youbot_manipulation", true);
	ROS_INFO("waiting for youbot_manipulation_server");
  ac->waitForServer();
  ROS_INFO("ready");

  youbot_manipulation::youbot_manipulationGoal goal;
 /* goal.object_name.data="AX-01b_bearing_box";
  ac->sendGoal(goal);
  ac->waitForResult();
  if(ac->getState()!=actionlib::SimpleClientGoalState::SUCCEEDED)
    std::cout<<"manipulation failed"<<std::endl;
    
  goal.object_name.data="AX-01_bearing_box";
  ac->sendGoal(goal);
  ac->waitForResult();
  if(ac->getState()!=actionlib::SimpleClientGoalState::SUCCEEDED)
    std::cout<<"manipulation failed"<<std::endl;
    
  goal.object_name.data="AX-03_axis";
  ac->sendGoal(goal);
  ac->waitForResult();
  if(ac->getState()!=actionlib::SimpleClientGoalState::SUCCEEDED)
    std::cout<<"manipulation failed"<<std::endl;*/
    
  goal.object_name.data="EM-02_coverplates_box";
  ac->sendGoal(goal);
  ac->waitForResult();
  if(ac->getState()!=actionlib::SimpleClientGoalState::SUCCEEDED)
    std::cout<<"manipulation failed"<<std::endl;
    
	return 0;
}
