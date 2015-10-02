#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "youbot_object_recognition/RecognizeObjectAction.h"

actionlib::SimpleActionClient<youbot_object_recognition::RecognizeObjectAction>* ac;

int main(int argc, char **argv){
	
	ros::init(argc, argv, "youbot_object_recognition_client");
  ros::NodeHandle nh;
	
	ac = new actionlib::SimpleActionClient<youbot_object_recognition::RecognizeObjectAction> ("kinect_recognize_object", true);
	ROS_INFO("waiting for kinect_recognize_object");
  ac->waitForServer();

  youbot_object_recognition::RecognizeObjectGoal goal;
  goal.object_name.data="all";
  ac->sendGoal(goal);
  ac->waitForResult();
  if(ac->getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
  {
		youbot_object_recognition::RecognizeObjectResultConstPtr result=ac->getResult();
		std::cout<<"object_list size: "<<result->object_list.objects.size()<<std::endl;
	}
  else
    std::cout<<"object recognition failed"<<std::endl;
	return 0;
}
