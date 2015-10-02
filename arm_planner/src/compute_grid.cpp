#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include "arm_planner/arm_planningAction.h"
#include "arm_planner/utils.h"

using namespace Eigen;

int main(int argc, char **argv){

  ros::init(argc, argv, "grid_request");
  ros::NodeHandle nh;

  actionlib::SimpleActionClient<arm_planner::arm_planningAction> ac("arm_planner", true);
  ac.waitForServer();
  arm_planner::arm_planningGoal goal;

  Eigen::Vector3d grid_center;
  float inValue, grid_pitch;
  std::cout << "type grid x " << std::endl;
  std::cin >> inValue;
  grid_center(0)=(inValue);
  std::cout << "type grid y " << std::endl;
  std::cin >> inValue;
  grid_center(1)=(inValue);
  std::cout << "type grid z " << std::endl;
  std::cin >> inValue;
  grid_center(2)=(inValue);

  std::cout << "type grid pitch " << std::endl;
  std::cin >> grid_pitch;
  float grid_pitch_rad=grid_pitch*M_PI/180;
  
  //double dist=grid_center.norm();
  Eigen::Vector3d rpy((0*M_PI/180.0), grid_pitch*M_PI/180, yawFromTarget(grid_center));
  Eigen::Affine3d A;
  Eigen::Matrix3d Rx; Rx<<1,0,0,0,cos(rpy(0)), -sin(rpy(0)),0, sin(rpy(0)), cos(rpy(0));
  Eigen::Matrix3d Rz; Rz<<cos(rpy(2)), -sin(rpy(2)),0, sin(rpy(2)), cos(rpy(2)),0, 0,0,1;
  Eigen::Matrix3d Ry; Ry<<cos(rpy(1)),0, sin(rpy(1)),0, 1,0, -sin(rpy(1)),0,cos(rpy(1));
  A.linear()=Rz*Ry*Rx;
  A.translation()=grid_center;
  Eigen::Matrix4d T=A.matrix();
 // std::cout<<T<<std::endl;

  float grid_size=.1; Eigen::Vector3d point; Eigen::Vector4d point_2;
  for(float step=-grid_size/2;step<grid_size/2;step+=.005)
  {
    point=Eigen::Vector3d(0, -grid_size/2,step);
    point_2=Eigen::Vector4d(point(0),point(1),point(2),1);
    point_2=T*point_2;
    point=point_2.head(3)/point_2(3);
    TargetRequest2RosGoal(point, grid_pitch, 90, 0, "...", goal);
   // std::cout<<point<<"\n"<<std::endl;
    ac.sendGoal(goal);
    ac.waitForResult();
    sleep(.5);
    
    point=Eigen::Vector3d(0, +grid_size/2,step);
    point_2=Eigen::Vector4d(point(0),point(1),point(2),1);
    point_2=T*point_2;
    point=point_2.head(3)/point_2(3);
    TargetRequest2RosGoal(point, grid_pitch, 90, 0, "...", goal);
   // std::cout<<point<<"\n"<<std::endl;
    ac.sendGoal(goal);
    ac.waitForResult();
    sleep(.5);
  }

  return 1;
}


