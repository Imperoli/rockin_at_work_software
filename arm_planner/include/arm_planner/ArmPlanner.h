#ifndef ARMPLANNER_HPP_
#define ARMPLANNER_HPP_

#include "arm_planner/planner.h"
#include "arm_planner/PlanningSceneFrame.h"
#include "arm_planner/utils.h"

#include <actionlib/server/simple_action_server.h>
#include "arm_planner/arm_planningAction.h"
#include "arm_planner/check_IK_feasibility.h"
#include "arm_planner/get_ee_pose.h"

#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>

// #include <youbot_driver_ros_interface/GripperPositions.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

namespace arm_planner{

using namespace Eigen;
using namespace std;

class ArmPlanner
{
public:
	ArmPlanner(ros::NodeHandle& nh, std::string name);
	void compute_action(const arm_planner::arm_planningGoalConstPtr & goal);
	void jointState2jointPositions(sensor_msgs::JointState& js, brics_actuator::JointPositions& jp);
	void set_planner_current_joint_state();
	void get_current_joint_state(Eigen::Matrix<double,5,1>& cjs);
	bool compute_approach_targetUP(Eigen::Vector3d& target, Eigen::Vector3d& pos_target);
	bool compute_approach_target_preferred_pitch(Eigen::Vector3d& target, Eigen::Vector3d& pos_target, double preferred_pitch);
	bool compute_approach_target(Eigen::Vector3d& target, Eigen::Vector3d& pos_target, double g_pitch);
	void target_request_cb (const arm_planner::TargetRequest::Ptr& msg);
	void obstacles_position_cb (const arm_planner::Obstacles::Ptr& msg);
	void appendObstacles(std::vector<Obstacle>& x,std::vector<Obstacle>& y);
	void jointstateCallback(const sensor_msgs::JointState::ConstPtr& msg);
	void initPlanning(char* work_space_dir);

	bool execute_plan(trajectory_msgs::JointTrajectory& traj/*, bool rotate_gripper*/);
	bool move_to_fixed_pose(Eigen::Matrix<double,5,1>& j_pos, double max_acc, double max_vel);
	bool simple_target_procedure(double max_acc, double max_vel);
	bool approaching_procedure(double max_acc, double max_vel);
	bool grasping_procedure(double max_acc, double max_vel);
	bool move_away_from_target(double max_acc, double max_vel);
	bool move_away_from_targetUP(double max_acc, double max_vel);
	bool close_gripper();
	bool open_gripper();
	bool grasp();
	bool drop();
	bool grasping_pipeline();
	bool dropping_pipeline();
	bool move_base_procedure();

	bool move_to_fixed_pose_cb(int pose_index);
	bool simple_target_cb();
	bool grasp_cb();
	bool drop_cb();

  bool get_ee_pose_cb(arm_planner::get_ee_pose::Request &req, arm_planner::get_ee_pose::Response &res);
  bool check_IK_feasibility_cb(arm_planner::check_IK_feasibility::Request &req, arm_planner::check_IK_feasibility::Response &res);
	bool open_gripper_cb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
	bool close_gripper_cb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
	bool grasping_failure();
	// void getFingerPosition(double* fingers);


	ros::ServiceClient finger_position_client;
	double finger_position[2];

	sensor_msgs::JointState current_jstate;
	brics_actuator::JointPositions start_pos,end_pos;
	brics_actuator::JointVelocities start_vel,end_vel;
	std::vector<std::string> joint_names;
	std::vector<Obstacle> obstacles;
	std::vector<Obstacle> scene_obstacles;
	std::vector<ControlPoint> control_points;
	std::vector<ControlPoint> scene_control_points, real_control_points;
	bool visualization;
	bool replan;
	bool replan_needed;
	bool stop_planning;
	bool is_executing;

	bool approaching;
	bool grasping;
	bool moving_to_fixed_pose;
	bool moving_away_from_target;

	int traj_index;
	int iter;
	Eigen::Vector3d target_cartesian_position, target_cartesian_normal, approach_target, grasping_target;
	Planner* planner;
	trajectory_msgs::JointTrajectory straight_traj, straight_traj_null;

	int incoming_req_mode;
	int req_mode; //0 free manipulation, 1 grasping, 2 dropping

	double gripper_roll, gripper_pitch, preferred_gripper_pitch;

	std::vector<Eigen::Vector3d> manipulation_area;
	std::vector<std::vector<Eigen::Vector3d> > workspace;
	std::vector<Eigen::Matrix<double,5,1> > fixed_poses; 
	Eigen::Matrix<double,5,1> target_joint_position;

	ros::Publisher arm_pub_pos;
	ros::Publisher gripper_pub_pos;
	ros::Publisher planning_scene_pub;
	
	ros::Publisher pub_js;
		
	actionlib::SimpleActionServer<arm_planner::arm_planningAction> planningAction;
	arm_planner::arm_planningResult result;
  	arm_planner::arm_planningFeedback feedback;

	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* move_base_client;
	
	Eigen::Matrix4d kinect_T, rgb_T;
	
	float velocity_scale;

};

}

#endif


