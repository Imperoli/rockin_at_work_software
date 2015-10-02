#ifndef PLANNER_HPP_
#define PLANNER_HPP_

#include "arm_planner/control_point.h"
#include "arm_planner/obstacle.h"
#include "arm_planner/utils.h"

#include "ros/ros.h"
#include "trajectory_generator/JStoJS.h"
#include "trajectory_generator/JStoCS.h"
#include "trajectory_generator/CStoCS.h"
#include "trajectory_generator/Circle.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "Eigen/Dense"
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointVelocities.h>
#include <geometry_msgs/Pose.h>
#include "rpg_youbot_common/rpg_youbot_common.h"
#include <sensor_msgs/JointState.h>
#include "ik_solver_service/SolvePreferredPitchIK.h"
#include "ik_solver_service/SolveClosestIK.h"

namespace arm_planner{

using namespace Eigen;

class Planner
{
public:
  Planner(){}
  Planner(ros::NodeHandle& nh);
  ~Planner();
  bool plan();
  bool plan_fixed_pose(Eigen::Matrix<double,5,1>& t_pos);
  bool plan_straight_traj(Eigen::Vector3d& st_pos, Eigen::Vector3d& t_pos, double pitch, double roll,trajectory_msgs::JointTrajectory& pl_traj);
  bool plan_straight_traj(Eigen::Matrix<double, 5, 1>& st_joints, Eigen::Vector3d& t_pos, double pitch, double roll, trajectory_msgs::JointTrajectory& pl_traj);
  void compute_control_points_position(std::vector<double>& joints_pos);
  void init_control_points();
  void init_fixed_obstacles();
  void compute_arm_joints_position(std::vector<double>& joints_pos);
  void init_arm_joints();
  void init_trajectory_service();
  bool in_collision(std::vector<double>& joints_pos);
  void compute_perturbation();
  void compute_perturbation_ee();
  bool compute_IK(Eigen::Vector3d& pos, Eigen::Vector3d& n);
  bool compute_IK(Eigen::Vector3d& pos, Eigen::Vector3d& n, Eigen::Matrix<double, 5, 1> &out_joints);
  void compute_ee_trajectory(trajectory_msgs::JointTrajectory& tr);
  bool smooth_trajectory_routine(trajectory_msgs::JointTrajectoryPoint& start, trajectory_msgs::JointTrajectoryPoint& end);
  bool smooth_trajectory();
  //inline void forwardKin(Eigen::Matrix<double,5,1>& j_pos, arm_planner::ControlPoint& c_point);
  //inline Eigen::VectorXd youbot2matlab(const Eigen::VectorXd j_pos);
  //inline Eigen::VectorXd matlab2youbot(const Eigen::VectorXd j_pos);
  void grows_trajectory(int factor, trajectory_msgs::JointTrajectory& traj);


  std::vector<Obstacle> obstacles;
  std::vector<Obstacle> fixed_obstacles;
  std::vector<ControlPoint> control_points;
  std::vector<ControlPoint> control_points_temp;
  std::vector<ControlPoint> arm_joints;
  int ee_index;
  int cam_index;
  std::vector<Eigen::Vector3d> ee_traj;
  //ros::ServiceClient cs2cs_client;
  ros::ServiceClient js2cs_client;
  ros::ServiceClient js2js_client;
  ros::ServiceClient cs2cs_client;
  trajectory_generator::CStoCS cs2cs;
  trajectory_generator::JStoCS js2cs;
  trajectory_generator::JStoJS js2js;
  trajectory_msgs::JointTrajectory planned_traj, temp_traj, smoothed_traj, temp_smoothed_traj, planned_straight_traj;
  brics_actuator::JointPositions start_pos;
  brics_actuator::JointPositions end_pos;
  brics_actuator::JointPositions target_pos;
  brics_actuator::JointVelocities start_vel;
  brics_actuator::JointVelocities end_vel;
  brics_actuator::JointVelocities target_vel;
  ik_solver_service::SolveClosestIK IK_srv;
  Eigen::Vector3d target_cartesian_position, target_cartesian_normal;
  Eigen::Matrix<double, 5, 1> current_joint_angles;
  ros::ServiceClient solve_ik_client;
  std::vector<trajectory_msgs::JointTrajectoryPoint> perturbations;
  /*geometry_msgs::Pose end_pos;
  double start_vel;
  double end_vel;*/
  double max_vel;
  double max_acc;
  double driver_loop_rate;
  double generator_loop_rate;
  int traj_growing_factor;
  bool feasible;
  bool help_routine;
  int rand_range, rand_offset;
};

}

#endif
