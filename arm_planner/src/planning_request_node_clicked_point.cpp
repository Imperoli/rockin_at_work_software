/*
 * planning_request.cpp
 *
 *      Author: Marco Imperoli
 */

#include "arm_planner/utils.h"
#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <arm_planner/arm_planningAction.h>
#include "geometry_msgs/PointStamped.h"


Eigen::Vector3d target_cartesian_position;

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


void point_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  actionlib::SimpleActionClient<arm_planner::arm_planningAction> ac("arm_planner", true);
  ac.waitForServer();
  arm_planner::arm_planningGoal goal;
  target_cartesian_position(0)=msg->point.x;
  target_cartesian_position(1)=msg->point.y;
  target_cartesian_position(2)=msg->point.z;
  int mode=1;
  gripper_pitch=80;
  gripper_roll=0;
  
    
   
       TargetRequest2RosGoal(target_cartesian_position, gripper_pitch, gripper_roll, mode, "rgb", goal);

        ac.sendGoal(goal);
}

int main(int argc, char **argv){

  ros::init(argc, argv, "planning_request_clicked_point");
    ros::NodeHandle nh;

  //ros::Subscriber sub = nh.subscribe("/planner_response", 1, response_cb);
  
  ros::Subscriber sub = nh.subscribe<geometry_msgs::PointStamped>("/clicked_point", 1, point_callback);

  ros::spin();

  return 1;
}


