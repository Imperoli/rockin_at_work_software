#include "arm_planner/utils.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

std::vector<std::string> joint_names;

bool jointState2jointPositions(const sensor_msgs::JointState::ConstPtr& js, Eigen::Matrix<double,5,1>& jp)
{
  int count=0;
  for (int j = 0; j < js->position.size(); j++)
  {
    for (int i = 0; i < 5; i++)
    {
      if (js->name[j] == joint_names[i])
      {
        jp(i) = js->position[j];
        count++;
      }
    }
  }
  if (count==5) return true;
  return false;
}

void jointstate_cb(const sensor_msgs::JointState::ConstPtr& msg, tf::TransformBroadcaster& br)
{
  Eigen::Matrix<double,5,1> j_pos;
  if(jointState2jointPositions(msg, j_pos))
  {
    tf::Transform transform;
    Eigen::Affine3d  cart_pos;
    Eigen::Matrix<double, 5,1> j_pos2;
    j_pos2=youbot2matlab(j_pos);
    forwardKinematicsEE(j_pos2, cart_pos);
    
  //  Eigen::Quaternionf q(Eigen::AngleAxisf(-6*M_PI/180, Vector3f::UnitZ()));
  //  std::cout<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<std::endl;
    
    tf::poseEigenToTF(cart_pos, transform);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "arm_base", "end_effector"));
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arm_frame_publisher");
  ros::NodeHandle nh;
  
  joint_names.push_back("arm_joint_1");
  joint_names.push_back("arm_joint_2");
  joint_names.push_back("arm_joint_3"); 
  joint_names.push_back("arm_joint_4"); 
  joint_names.push_back("arm_joint_5");
  tf::TransformBroadcaster br;

  ros::Subscriber joint_state_sub = nh.subscribe<sensor_msgs::JointState>("/joint_states", 1,boost::bind(jointstate_cb, _1, br));
  ros::spin();
  return 0;
}
