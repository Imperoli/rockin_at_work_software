#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "brics_actuator/JointVelocities.h"
#include <stdio.h>
using namespace std;

#define PS3_BUTTON_SELECT            0
#define PS3_BUTTON_STICK_LEFT        1
#define PS3_BUTTON_STICK_RIGHT       2
#define PS3_BUTTON_START             3
#define PS3_BUTTON_CROSS_UP          4
#define PS3_BUTTON_CROSS_RIGHT       5
#define PS3_BUTTON_CROSS_DOWN        6
#define PS3_BUTTON_CROSS_LEFT        7
#define PS3_BUTTON_REAR_LEFT_2       8
#define PS3_BUTTON_REAR_RIGHT_2      9
#define PS3_BUTTON_REAR_LEFT_1       10
#define PS3_BUTTON_REAR_RIGHT_1      11
#define PS3_BUTTON_ACTION_TRIANGLE   12
#define PS3_BUTTON_ACTION_CIRCLE     13
#define PS3_BUTTON_ACTION_CROSS      14
#define PS3_BUTTON_ACTION_SQUARE     15
#define PS3_BUTTON_PAIRING           16

#define PS3_AXIS_STICK_LEFT_LEFTWARDS    0
#define PS3_AXIS_STICK_LEFT_UPWARDS      1
#define PS3_AXIS_STICK_RIGHT_LEFTWARDS   2
#define PS3_AXIS_STICK_RIGHT_UPWARDS     3
#define PS3_AXIS_BUTTON_CROSS_UP         4
#define PS3_AXIS_BUTTON_CROSS_RIGHT      5
#define PS3_AXIS_BUTTON_CROSS_DOWN       6
#define PS3_AXIS_BUTTON_CROSS_LEFT       7
#define PS3_AXIS_BUTTON_REAR_LEFT_2      8
#define PS3_AXIS_BUTTON_REAR_RIGHT_2     9
#define PS3_AXIS_BUTTON_REAR_LEFT_1      10
#define PS3_AXIS_BUTTON_REAR_RIGHT_1     11
#define PS3_AXIS_BUTTON_ACTION_TRIANGLE  12
#define PS3_AXIS_BUTTON_ACTION_CIRCLE    13
#define PS3_AXIS_BUTTON_ACTION_CROSS     14
#define PS3_AXIS_BUTTON_ACTION_SQUARE    15
#define PS3_AXIS_ACCELEROMETER_LEFT      16
#define PS3_AXIS_ACCELEROMETER_FORWARD   17
#define PS3_AXIS_ACCELEROMETER_UP        18
#define PS3_AXIS_GYRO_YAW                19

#define LOGITECH_BUTTON_BLUE 0
#define LOGITECH_BUTTON_GREEN 1
#define LOGITECH_BUTTON_RED 2
#define LOGITECH_BUTTON_YELLOW 3
#define LOGITECH_BUTTON_LB 4
#define LOGITECH_BUTTON_RB 5
#define LOGITECH_BUTTON_LT 6
#define LOGITECH_BUTTON_RT 7
#define LOGITECH_BUTTON_BACK 8
#define LOGITECH_BUTTON_START 9


sensor_msgs::Joy joy;
ros::Publisher arm_pub;

int joint=0;


void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	  if(msg->buttons[LOGITECH_BUTTON_BLUE]==1){ joint=0;}
    else if(msg->buttons[LOGITECH_BUTTON_GREEN]==1){ joint=1;}
    else if(msg->buttons[LOGITECH_BUTTON_RED]==1){ joint=2;}
    else if(msg->buttons[LOGITECH_BUTTON_YELLOW]==1){ joint=3;}
    else if(msg->buttons[LOGITECH_BUTTON_RB]==1){ joint=4;}

    float value=msg->axes[PS3_AXIS_STICK_LEFT_UPWARDS]*1;
    brics_actuator::JointVelocities jvel;
    brics_actuator::JointValue jv; jv.unit = "s^-1 rad";
    jv.joint_uri="arm_joint_1"; jv.value= (joint==0)? value:0; jvel.velocities.push_back(jv); 
    jv.joint_uri="arm_joint_2"; jv.value= (joint==1)? value:0; jvel.velocities.push_back(jv);  
    jv.joint_uri="arm_joint_3"; jv.value= (joint==2)? value:0; jvel.velocities.push_back(jv); 
    jv.joint_uri="arm_joint_4"; jv.value= (joint==3)? value:0; jvel.velocities.push_back(jv); 
    jv.joint_uri="arm_joint_5"; jv.value= (joint==4)? value:0; jvel.velocities.push_back(jv); 

	  arm_pub.publish(jvel);
}


int main(int argc, char **argv)
{

	ros::init(argc, argv, "joystick_control_arm_only");

	ros::NodeHandle n;

	arm_pub = n.advertise<brics_actuator::JointVelocities>("/arm_1/arm_controller/velocity_command", 1);
	ros::Subscriber joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 1, joyCallback);
	//ros::spin();

  ros::spin();

	return 0;
}




