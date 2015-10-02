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
  std_msgs::String name;
  
  goal.mode=1;
  name.data="AX-01b_bearing_box";
  goal.object_name_list.push_back(name);
  //name.data="EM-01_aid_tray";
  //goal.object_name_list.push_back(name);
 /* name.data="AX-09_motor_with_gearbox";
  goal.object_name_list.push_back(name);
  name.data="AX-01b_bearing_box";
  goal.object_name_list.push_back(name);
  /*name.data="AX-01_bearing_box";
  goal.object_name_list.push_back(name);
  name.data="AX-03_axis";
  goal.object_name_list.push_back(name);
  name.data="AX-01b_bearing_box";
  goal.object_name_list.push_back(name);
  name.data="AX-01_bearing_box";
  goal.object_name_list.push_back(name);
  name.data="EM-01_aid_tray";
  goal.object_name_list.push_back(name);*/
  
  /*name.data="EM-02_coverplates_box";
  goal.object_name_list.push_back(name);*/

  ac->sendGoal(goal);
  ac->waitForResult();
  if(ac->getState()!=actionlib::SimpleClientGoalState::SUCCEEDED)
    std::cout<<"manipulation failed"<<std::endl;
    
  /*youbot_manipulation::youbot_manipulationGoal goal2;
  goal2.mode=2;
  //name.data="AX-01_bearing_box";
  //goal.object_name_list.push_back(name);
  name.data="EM-01_aid_tray";
  goal2.object_name_list.push_back(name);
  ac->sendGoal(goal2);
  ac->waitForResult();
  if(ac->getState()!=actionlib::SimpleClientGoalState::SUCCEEDED)
    std::cout<<"manipulation failed"<<std::endl;
    */
  youbot_manipulation::youbot_manipulationGoal goal3;
  goal3.mode=1;
  name.data="AX-01_bearing_box";
  goal3.object_name_list.push_back(name);
  ac->sendGoal(goal3);
  ac->waitForResult();
  
  youbot_manipulation::youbot_manipulationGoal goal4;
  goal4.mode=1;
  name.data="AX-03_axis";
  goal4.object_name_list.push_back(name);
  ac->sendGoal(goal4);
  ac->waitForResult();

    
	return 0;
}
