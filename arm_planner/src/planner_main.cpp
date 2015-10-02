/*
 * planner_main.cpp
 *
 *      Author: Marco Imperoli
 */


#include "arm_planner/ArmPlanner.h"


namespace arm_planner{
using namespace std;

ArmPlanner::ArmPlanner(ros::NodeHandle& nh, std::string name):
planningAction(nh, "arm_planner", boost::bind(&ArmPlanner::compute_action, this, _1), false)
{
  //visualization=true;
  nh.param("visualization", visualization, true);
  replan=false;
  replan_needed=false;
  stop_planning=false;
  is_executing=false;

  approaching=false;
  grasping=false;
  moving_to_fixed_pose=false;
  moving_away_from_target=false;
  
  velocity_scale=1.5;

  traj_index=0;
  iter=0;

  req_mode=0; //0 free manipulation, 1 grasping, 2 dropping

  move_base_client=new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base", true);

  //wait for the action server to come up
  move_base_client->waitForServer(ros::Duration(5.0));
  //while(!move_base_client->waitForServer(ros::Duration(5.0))){
  //  ROS_INFO("Waiting for the move_base action server to come up");
  //}


  gripper_roll=0;
  planningAction.start();
  // getFingerPosition(finger_position);
}

void ArmPlanner::jointState2jointPositions(sensor_msgs::JointState& js, brics_actuator::JointPositions& jp)
{
  for (int j = 0; j < js.position.size(); j++)
    {
      for (int i = 0; i < 5; i++)
      {
        if (js.name[j] == joint_names[i])
        {
    jp.positions[i].value = js.position[j];
        }
      }
    }
}

void ArmPlanner::set_planner_current_joint_state(){

  planner->current_joint_angles(0)=(start_pos.positions[0].value);
  planner->current_joint_angles(1)=(start_pos.positions[1].value);
  planner->current_joint_angles(2)=(start_pos.positions[2].value);
  planner->current_joint_angles(3)=(start_pos.positions[3].value);
  planner->current_joint_angles(4)=(start_pos.positions[4].value);
}

void ArmPlanner::get_current_joint_state(Eigen::Matrix<double,5,1>& cjs){

  cjs(0)=(start_pos.positions[0].value);
  cjs(1)=(start_pos.positions[1].value);
  cjs(2)=(start_pos.positions[2].value);
  cjs(3)=(start_pos.positions[3].value);
  cjs(4)=(start_pos.positions[4].value);
}


bool ArmPlanner::compute_approach_targetUP(Eigen::Vector3d& target, Eigen::Vector3d& pos_target){
  pos_target(0)=target(0); pos_target(1)=target(1); pos_target(2)=target(2)+.04;
  planner->max_acc=0.4;
  planner->max_vel=0.04*velocity_scale;
  if(planner->plan_straight_traj(pos_target, target, gripper_pitch, gripper_roll, straight_traj_null)){
    int i=0;//straight_traj_null.points.size()-1;
    target_joint_position(0)=straight_traj_null.points[i].positions[4];
    target_joint_position(1)=straight_traj_null.points[i].positions[3];
    target_joint_position(2)=straight_traj_null.points[i].positions[2];
    target_joint_position(3)=straight_traj_null.points[i].positions[1];
    target_joint_position(4)=straight_traj_null.points[i].positions[0];
    return true;
  }
  //return false;
  for(int iter=0; iter<20; iter++){
    int range=800; int offset=400; int range_z=200;
    double pert0=(double)(rand()%range-offset)/10000.0;
    double pert1=(double)(rand()%range-offset)/10000.0;
    double pert2=(double)(rand()%range)/10000.0;
    pos_target(0)=target(0)+pert0; pos_target(1)=target(1)+pert1; pos_target(2)=target(2)+pert2;
    if(planner->plan_straight_traj(pos_target, target, gripper_pitch, gripper_roll, straight_traj_null)){
      int i=0;//straight_traj_null.points.size()-1;
      target_joint_position(0)=straight_traj_null.points[i].positions[4];
      target_joint_position(1)=straight_traj_null.points[i].positions[3];
      target_joint_position(2)=straight_traj_null.points[i].positions[2];
      target_joint_position(3)=straight_traj_null.points[i].positions[1];
      target_joint_position(4)=straight_traj_null.points[i].positions[0];
      return true;
    }
  }
  pos_target=Eigen::Vector3d(0,0,0);
  return false;
}

bool ArmPlanner::compute_approach_target_preferred_pitch(Eigen::Vector3d& target, Eigen::Vector3d& pos_target, double preferred_pitch){
  if(compute_approach_target(target, pos_target,preferred_pitch)){gripper_pitch=preferred_pitch; return true;}
  double step=5;
  while((preferred_pitch+step)<=90||(preferred_pitch-step)>=0){
    if(compute_approach_target(target, pos_target, preferred_pitch-step)){gripper_pitch=preferred_pitch-step; return true;}
    if(compute_approach_target(target, pos_target, preferred_pitch+step)){gripper_pitch=preferred_pitch+step; return true;}
    step+=5;
  }
  gripper_pitch=preferred_pitch;
  return false;
}

bool ArmPlanner::compute_approach_target(Eigen::Vector3d& target, Eigen::Vector3d& pos_target, double g_pitch){
  double a=.07; double yaw=yawFromTarget(target);
  double pitch=g_pitch*M_PI/180.0;
  Eigen::Vector3d displacement(-cos(pitch)*cos(yaw), -cos(pitch)*sin(yaw), sin(pitch));
  pos_target=target+(a*displacement);
  planner->max_acc=0.4;
  planner->max_vel=0.04*velocity_scale;
  if(planner->plan_straight_traj(pos_target, target, g_pitch, gripper_roll, straight_traj_null)){
    int i=0;//straight_traj_null.points.size()-1;
    target_joint_position(0)=straight_traj_null.points[i].positions[4];
    target_joint_position(1)=straight_traj_null.points[i].positions[3];
    target_joint_position(2)=straight_traj_null.points[i].positions[2];
    target_joint_position(3)=straight_traj_null.points[i].positions[1];
    target_joint_position(4)=straight_traj_null.points[i].positions[0];
    return true;
  }
  //return false;
  for(int iter=0; iter<10; iter++){
    int range=200; int offset=100;
    double pert_pitch=(double)(rand()%range-offset)/2000.0; 
    double pert_yaw=(double)(rand()%range-offset)/2000.0;
    double pert_a=(double)(rand()%range)/5000.0;
    displacement(0)=-cos(pitch+pert_pitch)*cos(yaw+pert_yaw); displacement(1)=-cos(pitch+pert_pitch)*sin(yaw+pert_yaw); displacement(2)=sin(pitch+pert_pitch);
    pos_target=target+((a+pert_a)*displacement);
    if(planner->plan_straight_traj(pos_target, target, g_pitch, gripper_roll, straight_traj_null)){
      int i=0;//straight_traj_null.points.size()-1;
      target_joint_position(0)=straight_traj_null.points[i].positions[4];
      target_joint_position(1)=straight_traj_null.points[i].positions[3];
      target_joint_position(2)=straight_traj_null.points[i].positions[2];
      target_joint_position(3)=straight_traj_null.points[i].positions[1];
      target_joint_position(4)=straight_traj_null.points[i].positions[0];
      return true;
    }
  }
  pos_target=Eigen::Vector3d(0,0,0);
  return false;
}

/*void ArmPlanner::target_request_cb (const arm_planner::TargetRequest::Ptr& msg)
{  
  Eigen::Matrix<double,5,1> cjs;
  get_current_joint_state(cjs);
  rosMsg2TargetRequest(*msg, kinect_T, rgb_T, cjs, target_cartesian_position, target_cartesian_normal, gripper_pitch, gripper_roll);
  
  if(is_executing){
    if(req_mode==0){
      double eps_target=.005;
      double t1t2_dist=(target_cartesian_position-planner->target_cartesian_position).norm();
      Eigen::Vector3d ee_pos;
      compute_ee_position(planner->current_joint_angles, ee_pos);
      double eet1_dist=(ee_pos-target_cartesian_position).norm();
      if((eet1_dist/t1t2_dist<2)&&t1t2_dist>eps_target){
        std::cout<<"replan (target)"<<std::endl;
        replan_needed=true;
        return;
      }
    }
    if(req_mode==1||req_mode==2){
      if(approaching){
        double t1t2_dist=(target_cartesian_position-grasping_target).norm();
        if(t1t2_dist>.03){
          Eigen::Vector3d approach;
          if(!compute_approach_target_preferred_pitch(target_cartesian_position, approach, preferred_pitch)) return;
          Eigen::Vector3d ee_pos;
          compute_ee_position(planner->current_joint_angles, ee_pos);
          double eeappr_dist=(ee_pos-approach_target).norm();
          double appr1appr2_dist=(approach_target-approach).norm();
          if(eeappr_dist/appr1appr2_dist<2){
            std::cout<<"replan (target)"<<std::endl;
            approach_target=approach;
            grasping_target=target_cartesian_position;
            replan_needed=true;
            return;
          }
        }
      }
      if(grasping){
        Eigen::Vector3d ee_pos;
        compute_ee_position(planner->current_joint_angles, ee_pos);
        if((ee_pos-grasping_target).norm()<.05) return;

        double t1grasp_dist=(target_cartesian_position-grasping_target).norm();
        if(t1grasp_dist>.02){
          std::cout<<"replan (target)"<<std::endl;
          grasping_target=target_cartesian_position;
          replan_needed=true;
          return;
        }
      }
    }
  }
  else if(grasping){
    Eigen::Vector3d ee_pos;
    compute_ee_position(planner->current_joint_angles, ee_pos);
    if((ee_pos-grasping_target).norm()<.05) return;

    double t1grasp_dist=(target_cartesian_position-grasping_target).norm();
    if(t1grasp_dist>.02){
      std::cout<<"replan2 (target)"<<std::endl;
      grasping_target=target_cartesian_position;
      replan_needed=true;
      replan=true;
      return;
    }
  }
}*/

void ArmPlanner::obstacles_position_cb (const arm_planner::Obstacles::Ptr& msg)
{  
  rosMsg2Obstacles(*msg, kinect_T, obstacles);

  if(is_executing){
    //if(req_mode==0||req_mode==1||req_mode==2){
      trajectory_msgs::JointTrajectoryPoint traj_point;
      int step=ceil(planner->driver_loop_rate/30); 
      //int traj_point_distance;
      if(req_mode==0||approaching||moving_to_fixed_pose){
        for(int i=traj_index+1;i<planner->planned_traj.points.size();i+=step){
          traj_point = planner->planned_traj.points[i];
          if(check_obstacles_collision(control_points, obstacles, traj_point.positions)&&(i-traj_index)<(2*planner->driver_loop_rate)){
            std::cout<<"replan (collision)"<<std::endl;
            replan_needed=true;
            return;
          }
        }
      }
      if(grasping||moving_away_from_target){
        for(int i=traj_index+1;i<straight_traj.points.size();i+=step){
          traj_point = straight_traj.points[i];
          if(check_obstacles_collision(control_points, obstacles, traj_point.positions)){
            std::cout<<"emergency stop (collision)"<<std::endl;
            stop_planning=true;
            return;
          }
        }
      }
    //}
  }
}


void ArmPlanner::appendObstacles(std::vector<Obstacle>& x,std::vector<Obstacle>& y){
  for (int i=0;i<x.size();i++){
    y.push_back(x[i]);
  }
}

void ArmPlanner::jointstateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  current_jstate=*msg;
  jointState2jointPositions(current_jstate, start_pos);
}


void ArmPlanner::initPlanning(char* work_space_dir){
  joint_names.push_back("arm_joint_1");
  joint_names.push_back("arm_joint_2");
  joint_names.push_back("arm_joint_3"); 
  joint_names.push_back("arm_joint_4"); 
  joint_names.push_back("arm_joint_5");
  brics_actuator::JointValue jv; jv.unit="rad";
  jv.joint_uri="arm_joint_1"; start_pos.positions.push_back(jv); 
  jv.joint_uri="arm_joint_2"; start_pos.positions.push_back(jv); 
  jv.joint_uri="arm_joint_3"; start_pos.positions.push_back(jv); 
  jv.joint_uri="arm_joint_4"; start_pos.positions.push_back(jv); 
  jv.joint_uri="arm_joint_5"; start_pos.positions.push_back(jv); 
  
  workspace.clear();
  std::string dir(work_space_dir);
  loadWorkSpace( dir,workspace);
  loadMatrix(dir, kinect_T, rgb_T);


  //Eigen::Matrix<double,5,1> look_in_front_45c; look_in_front_45c<<2.94961, 0.68896, -1.40066, 3.54314, 1.35263;
  Eigen::Matrix<double,5,1> look_in_front_45c; look_in_front_45c<<2.94961, 0.58896, -1.40066, 3.54314, 1.35263;
  Eigen::Matrix<double,5,1> look_in_front_45a; look_in_front_45a<<2.94961, 0.58896, -1.40066, 3.54314, 1.35263;
  Eigen::Matrix<double,5,1> look_in_front_45b; look_in_front_45b<<2.94961, 0.260567, -1.02419, 3.49506, .013;
  Eigen::Matrix<double,5,1> look_in_front_75a; look_in_front_75a<<2.94961, 1.06367, -1.35692, 3.54829, 2.92;
  Eigen::Matrix<double,5,1> look_in_front_75b; look_in_front_75b<<2.94961, 1.06367, -1.35692, 3.54829, .013;
  Eigen::Matrix<double,5,1> look_in_front_30a; look_in_front_30a<<2.94961, 0.125903, -0.595511, 2.93925, 2.92;
  Eigen::Matrix<double,5,1> look_in_front_30b; look_in_front_30b<<2.94961, 0.125903, -0.595511, 2.93925, .013;
  Eigen::Matrix<double,5,1> look_in_front_20a; look_in_front_20a<<2.94961, 0.0191032, -0.583662, 2.85967, 2.92;
  Eigen::Matrix<double,5,1> look_in_front_20b; look_in_front_20b<<2.94961, 0.0191032, -0.74662, 2.65, 1.46;

  /*Eigen::Matrix<double,5,1> home1; home1<<.011,.011,-.16,.023,.012;
  Eigen::Matrix<double,5,1> home2; home2<<2.94,.011,-.16,.023,2.92;
  Eigen::Matrix<double,5,1> up1; up1<<2.94,1.2,-2.5,1.65,2.92;
  Eigen::Matrix<double,5,1> up2; up2<<.011,1.2,-2.5,1.65,2.92;*/
  fixed_poses.clear();
  fixed_poses.push_back(look_in_front_45c); fixed_poses.push_back(look_in_front_45a); fixed_poses.push_back(look_in_front_45b);
  fixed_poses.push_back(look_in_front_75a); fixed_poses.push_back(look_in_front_75b);
  fixed_poses.push_back(look_in_front_30a); fixed_poses.push_back(look_in_front_30b);
  fixed_poses.push_back(look_in_front_20a); fixed_poses.push_back(look_in_front_20b);

  /*fixed_poses.push_back(home1); fixed_poses.push_back(home2); 
  fixed_poses.push_back(up1); fixed_poses.push_back(up2); */
}

bool ArmPlanner::execute_plan(trajectory_msgs::JointTrajectory& traj/*, bool rotate_gripper*/){
  is_executing=true;
  ros::Rate lr(planner->driver_loop_rate);
  ROS_INFO("EXECUTING PLAN...");
  trajectory_msgs::JointTrajectoryPoint point;
  double first_pt[5];
  traj_index=0;
  while(traj_index<traj.points.size()&&!stop_planning)
  {
    point = traj.points[traj_index];

    if(replan_needed){
      iter=0;
      replan=true;
      break;
    }

    int j = 0;
    while (!point.positions.empty())
    {
      first_pt[j] = point.positions.back();
      j++;
      point.positions.pop_back();
      /*if(j==4&&!rotate_gripper){
        first_pt[j] =planner->current_joint_angles(4);
        break;
      }*/
    }
    arm_pub_pos.publish(rpg_youbot_common::generateJointPositionMsg(first_pt));

    //// for webots
    planner->current_joint_angles(0)=first_pt[0];
    planner->current_joint_angles(1)=first_pt[1];
    planner->current_joint_angles(2)=first_pt[2];
    planner->current_joint_angles(3)=first_pt[3];
    planner->current_joint_angles(4)=first_pt[4];
    
    /*sensor_msgs::JointState js;
    js.name=joint_names;
    js.position.push_back(first_pt[0]);js.position.push_back(first_pt[1]);js.position.push_back(first_pt[2]);js.position.push_back(first_pt[3]);js.position.push_back(first_pt[4]);
    pub_js.publish(js);*/
    //////////////////////

    traj_index++;

    lr.sleep();
  }
  is_executing=false;
  if(traj_index==traj.points.size()){
  /*  if(!rotate_gripper){
      first_pt[0]=planner->current_joint_angles(0);
      first_pt[1]=planner->current_joint_angles(1);
      first_pt[2]=planner->current_joint_angles(2);
      first_pt[3]=planner->current_joint_angles(3);
      first_pt[4]=(-gripper_roll*M_PI/180.0)+joint_offsets[4];
      arm_pub_pos.publish(rpg_youbot_common::generateJointPositionMsg(first_pt));
      sleep(1);
      planner->current_joint_angles(4)=first_pt[4];
    }*/
    traj_index=0;
    ROS_INFO("GOAL REACHED!!!");
    return true;
  }

  traj_index=0;
  return false;
}

bool ArmPlanner::move_to_fixed_pose(Eigen::Matrix<double,5,1>& j_pos, double max_acc, double max_vel){
  moving_to_fixed_pose=true;
  iter=0;
  replan=true;
  while(iter<10&&!stop_planning){
    if(replan){

      //replan=false;
      replan_needed=false;
      planner->planned_traj.points.clear();
  

      //planner->target_cartesian_position=t_pos;
      //planner->target_cartesian_position=approach_target;
      //planner->target_cartesian_normal=t_norm;
      planner->obstacles=obstacles;
      planner->max_acc=max_acc;
      planner->max_vel=max_vel;

        ros::Time tic=ros::Time::now();

      if(planner->plan_fixed_pose(j_pos)){

        ros::Time toc=ros::Time::now();
        double elapsed_secs = (toc-tic).toSec();
        cout<<"elapsed secs "<<elapsed_secs<<endl;

        if(stop_planning){
          iter=0;
          replan=false;
          moving_to_fixed_pose=false;
          return false;
        }

        if (planner->feasible)
          {
          //iter=0;
          if(execute_plan(planner->planned_traj)){
            iter=0;
            replan=false;
            moving_to_fixed_pose=false;
            return true;        
          }
          }else{
          iter=0;
          replan=false;
          ROS_WARN("planning not feasible");
          moving_to_fixed_pose=false;
          return false;
        }//end if planner feasible
      }else{
        ROS_WARN("plan not found!!!");
        iter++;
        if(iter>10){
          iter=0;
          replan=false;
          moving_to_fixed_pose=false;
          return false;
        }else{
          is_executing=true;
          replan=true;
          ros::Rate rate(30);
          rate.sleep();
          is_executing=false;
        }
      }//end if plan()
    }//end if replan

    //ros::Rate rate(50);
    //rate.sleep();
  }
  iter=0;
  replan=false;
  moving_to_fixed_pose=false;
  return false;
}

bool ArmPlanner::simple_target_procedure(double max_acc, double max_vel){
  iter=0;
  replan=true;
  while(iter<1&&!stop_planning){
    if(replan){

      //replan=false;
      replan_needed=false;
      planner->planned_traj.points.clear();
  

      planner->target_cartesian_position=target_cartesian_position;
      planner->target_cartesian_normal=target_cartesian_normal;
      planner->obstacles=obstacles;
      planner->max_acc=max_acc;
      planner->max_vel=max_vel;

        ros::Time tic=ros::Time::now();

      if(planner->plan()){

        ros::Time toc=ros::Time::now();
        double elapsed_secs = (toc-tic).toSec();
        cout<<"elapsed secs "<<elapsed_secs<<endl;

        if(stop_planning){
          iter=0;
          replan=false;
          return false;
        }

        if (planner->feasible)
          {
          //iter=0;
          if(execute_plan(planner->planned_traj)){
            iter=0;
            replan=false;
            return true;        
          }
          }else{
          iter=0;
          replan=false;
          ROS_WARN("planning not feasible");
          return false;
        }//end if planner feasible
      }else{
        ROS_WARN("plan not found!!!");
        iter++;
        if(iter>1){
          iter=0;
          replan=false;
          return false;
        }else{
          is_executing=true;
          replan=true;
          ros::Rate rate(30);
          rate.sleep();
          is_executing=false;
        }
      }//end if plan()
    }//end if replan

    //ros::Rate rate(50);
    //rate.sleep();
  }
  iter=0;
  replan=false;
  return false;
}

bool ArmPlanner::approaching_procedure(double max_acc, double max_vel){
  approaching=true;
  iter=0;
  replan=true;
  while(iter<10&&!stop_planning){
    if(replan){

      //replan=false;
      replan_needed=false;
      planner->planned_traj.points.clear();
  

      //planner->target_cartesian_position=target_cartesian_position;
      //planner->target_cartesian_position=approach_target;
      //planner->target_cartesian_normal=target_cartesian_normal;
      //planner->obstacles=obstacles;
      //planner->max_acc=max_acc;
      //planner->max_vel=max_vel;

        ros::Time tic=ros::Time::now();

      //if(planner->plan()){
      if(planner->plan_fixed_pose(target_joint_position)){

        ros::Time toc=ros::Time::now();
        double elapsed_secs = (toc-tic).toSec();
        cout<<"elapsed secs "<<elapsed_secs<<endl;

        if(stop_planning){
          iter=0;
          replan=false;
          approaching=false;
          return false;
        }

        if (planner->feasible)
          {
          //iter=0;
          if(execute_plan(planner->planned_traj)){
            iter=0;
            replan=false;
            approaching=false;
            return true;        
          }
          }else{
          iter=0;
          replan=false;
          ROS_WARN("planning not feasible");
          approaching=false;
          return false;
        }//end if planner feasible
      }else{
        ROS_WARN("plan not found!!!");
        iter++;
        if(iter>10){
          iter=0;
          replan=false;
          approaching=false;
          return false;
        }else{
          is_executing=true;
          replan=true;
          ros::Rate rate(30);
          rate.sleep();
          is_executing=false;
        }
      }//end if plan()
    }//end if replan

    //ros::Rate rate(50);
    //rate.sleep();
  }
  iter=0;
  replan=false;
  approaching=false;
  return false;
}

bool ArmPlanner::grasping_procedure(double max_acc, double max_vel){
  grasping=true;
  iter=0;
  replan=true;
  while(iter<10&&!stop_planning){
    if(replan){

      //replan=false;
      replan_needed=false;
      straight_traj.points.clear();
      planner->planned_traj.points.clear();
  

    //  planner->target_cartesian_position=grasping_target;
      //planner->target_cartesian_position=approach_target;
    //  planner->target_cartesian_normal=target_cartesian_normal;
      planner->obstacles=obstacles;
      planner->max_acc=max_acc;
      planner->max_vel=max_vel;

        ros::Time tic=ros::Time::now();

      if(planner->plan_straight_traj(planner->current_joint_angles, grasping_target, gripper_pitch, gripper_roll, straight_traj)){

        ros::Time toc=ros::Time::now();
        double elapsed_secs = (toc-tic).toSec();
        cout<<"elapsed secs "<<elapsed_secs<<endl;

        if(stop_planning){
          iter=0;
          replan=false;
          grasping=false;
          return false;
        }

        if(execute_plan( /*planner->planned_traj*/straight_traj)){
          iter=0;
          replan=false;
          grasping=false;
          return true;        
        }
      }else{
        ROS_WARN("plan not found!!!");
        iter++;
        if(iter>10){
          iter=0;
          replan=false;
          grasping=false;
          return false;
        }else{
          is_executing=true;
          replan=true;
          ros::Rate rate(30);
          rate.sleep();
          is_executing=false;
        }
      }//end if plan()
    }//end if replan

    //ros::Rate rate(50);
    //rate.sleep();
  }
  iter=0;
  replan=false;
  grasping=false;
  return false;
}

bool ArmPlanner::move_away_from_target(double max_acc, double max_vel){
  moving_away_from_target=true;

  Eigen::Vector3d pos_target;
  Eigen::Vector3d ee_pos;
  compute_ee_position(planner->current_joint_angles, ee_pos);
  double a=.05; double yaw=yawFromTarget(ee_pos);
  double pitch=gripper_pitch*M_PI/180.0;

  Eigen::Vector3d displacement(-cos(pitch)*cos(yaw), -cos(pitch)*sin(yaw), sin(pitch));
  pos_target=ee_pos+(a*displacement);
  //pos_target(0)=ee_pos(0); pos_target(1)=ee_pos(1); pos_target(2)=ee_pos(2)+.05;

  iter=0;
  replan=true;
  while(iter<10&&!stop_planning){
    if(replan){
      //replan=false;
      replan_needed=false;
      straight_traj.points.clear();


    //  planner->target_cartesian_position=pos_target;
      //planner->target_cartesian_position=approach_target;
    //  planner->target_cartesian_normal=target_cartesian_normal;
      planner->obstacles=obstacles;
      planner->max_acc=max_acc;
      planner->max_vel=max_vel;

        ros::Time tic=ros::Time::now();

      if(planner->plan_straight_traj(planner->current_joint_angles, pos_target, gripper_pitch, gripper_roll, straight_traj)){

        ros::Time toc=ros::Time::now();
        double elapsed_secs = (toc-tic).toSec();
        cout<<"elapsed secs "<<elapsed_secs<<endl;

        if(stop_planning){
          iter=0;
          replan=false;
          moving_away_from_target=false;
          return false;
        }

        if(execute_plan( straight_traj)){
          iter=0;
          replan=false;
          moving_away_from_target=false;
          return true;        
        }
      }else{
        ROS_WARN("plan not found!!!");
        iter++;
        if(iter>10){
          iter=0;
          replan=false;
          moving_away_from_target=false;
          return false;
        }else{
          is_executing=true;
          replan=true;
          ros::Rate rate(30);
          rate.sleep();
          int range=800; int offset=400; int range_z=200;
          double pert0=(double)(rand()%range-offset)/10000.0;
          double pert1=(double)(rand()%range-offset)/10000.0;
          double pert2=(double)(rand()%range)/10000.0;
          compute_ee_position(planner->current_joint_angles, ee_pos);
          pos_target(0)=ee_pos(0)+pert0; pos_target(1)=ee_pos(1)+pert1; pos_target(2)=ee_pos(2)+.05+pert2;
          is_executing=false;
        }
      }//end if plan()
    }//end if replan

    //ros::Rate rate(50);
    //rate.sleep();
  }
  iter=0;
  replan=false;
  moving_away_from_target=false;
  return false;
}


bool ArmPlanner::move_away_from_targetUP(double max_acc, double max_vel){
  moving_away_from_target=true;

  Eigen::Vector3d pos_target;
Eigen::Vector3d ee_pos;
  compute_ee_position(planner->current_joint_angles, ee_pos);
  double a=.05; double yaw=yawFromTarget(ee_pos);
  double pitch=gripper_pitch*M_PI/180.0;

  //Eigen::Vector3d displacement(-cos(pitch)*cos(yaw), -cos(pitch)*sin(yaw), sin(pitch));
  //pos_target=ee_pos+(a*displacement);
  pos_target(0)=ee_pos(0); pos_target(1)=ee_pos(1); pos_target(2)=ee_pos(2)+.04;

  iter=0;
  replan=true;
  while(iter<10&&!stop_planning){
    if(replan){
      //replan=false;
      replan_needed=false;
      straight_traj.points.clear();


      //planner->target_cartesian_position=pos_target;
      //planner->target_cartesian_normal=target_cartesian_normal;
      planner->obstacles=obstacles;
      planner->max_acc=max_acc;
      planner->max_vel=max_vel;

        ros::Time tic=ros::Time::now();

      if(planner->plan_straight_traj(planner->current_joint_angles, pos_target, gripper_pitch, gripper_roll, straight_traj)){

        ros::Time toc=ros::Time::now();
        double elapsed_secs = (toc-tic).toSec();
        cout<<"elapsed secs "<<elapsed_secs<<endl;

        if(stop_planning){
          iter=0;
          replan=false;
          moving_away_from_target=false;
          return false;
        }

        if(execute_plan( straight_traj)){
          iter=0;
          replan=false;
          moving_away_from_target=false;
          return true;        
        }
      }else{
        ROS_WARN("plan not found!!!");
        iter++;
        if(iter>10){
          iter=0;
          replan=false;
          moving_away_from_target=false;
          return false;
        }else{
          is_executing=true;
          replan=true;
          ros::Rate rate(30);
          rate.sleep();
          int range=800; int offset=400; int range_z=200;
          double pert0=(double)(rand()%range-offset)/10000.0;
          double pert1=(double)(rand()%range-offset)/10000.0;
          double pert2=(double)(rand()%range)/10000.0;
          compute_ee_position(planner->current_joint_angles, ee_pos);
          pos_target(0)=ee_pos(0)+pert0; pos_target(1)=ee_pos(1)+pert1; pos_target(2)=ee_pos(2)+.05+pert2;
          is_executing=false;
        }
      }//end if plan()
    }//end if replan

    //ros::Rate rate(50);
    //rate.sleep();
  }
  iter=0;
  replan=false;
  moving_away_from_target=false;
  return false;
}

// void ArmPlanner::getFingerPosition(double* fingers){
//   youbot_driver_ros_interface::GripperPositions srv;
//   if(finger_position_client.call(srv)){
//     fingers[0]=srv.response.left_position;
//     fingers[1]=srv.response.right_position;
//   }else{
//     ROS_WARN("could not call gripper_positions service");
//     fingers[0]=.0091;
//     fingers[1]=.0091;
//   }
//   std::cout<<"finger_l "<<fingers[0]<<"\nfinger_r "<<fingers[1]<<std::endl;
// }

bool ArmPlanner::grasping_failure(){
  return false;
}

bool ArmPlanner::close_gripper(){
  brics_actuator::JointValue gripperJointPositionL;
  brics_actuator::JointValue gripperJointPositionR;
  gripperJointPositionL.joint_uri = "gripper_finger_joint_l";
  gripperJointPositionL.value = 0.01149;
  gripperJointPositionL.unit = boost::units::to_string(boost::units::si::meter);

  gripperJointPositionR.joint_uri = "gripper_finger_joint_r";
  gripperJointPositionR.value = 0.01149;
  gripperJointPositionR.unit = boost::units::to_string(boost::units::si::meter);

  brics_actuator::JointPositions command;
  command.positions.push_back(gripperJointPositionL);
  command.positions.push_back(gripperJointPositionR);
  gripper_pub_pos.publish(command);
  sleep(2);
  // getFingerPosition(finger_position);
  return true;  
}

bool ArmPlanner::open_gripper(){
  brics_actuator::JointValue gripperJointPositionL;
  brics_actuator::JointValue gripperJointPositionR;
  gripperJointPositionL.joint_uri = "gripper_finger_joint_l";
  gripperJointPositionL.value = 0;
  gripperJointPositionL.unit = boost::units::to_string(boost::units::si::meter);

  gripperJointPositionR.joint_uri = "gripper_finger_joint_r";
  gripperJointPositionR.value = 0;
  gripperJointPositionR.unit = boost::units::to_string(boost::units::si::meter);

  brics_actuator::JointPositions command;
  command.positions.push_back(gripperJointPositionL);
  command.positions.push_back(gripperJointPositionR);
  gripper_pub_pos.publish(command);
  sleep(1);
  // getFingerPosition(finger_position);
  return true;  
}

bool ArmPlanner::grasp(){
  if(close_gripper()){
    //if(!grasping_failure()){
      return true;
    //}else{
      //recovery
    //}
  }
  return false;
}

bool ArmPlanner::drop(){
  if(open_gripper()){
    return true;
  }
  return false;
}

bool ArmPlanner::move_base_procedure(){
  
  compute_manipulation_area(workspace,target_cartesian_position,preferred_gripper_pitch,0,manipulation_area);
  Eigen::Vector3d base_target;
  if(!compute_base_target(manipulation_area, base_target)){
    result.move_base=false;
    return false;
  }

  result.move_base=true;
  result.base_target.position.x=base_target(0);
  result.base_target.position.y=base_target(1);
  result.base_target.position.z=0;
  result.base_target.orientation.x=0;
  result.base_target.orientation.y=0;
  result.base_target.orientation.z=0;
  result.base_target.orientation.w=1.0;

  return true;


  /*move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = base_target(0);
  goal.target_pose.pose.position.y = base_target(1);
  goal.target_pose.pose.orientation.w = 1.0;

  if(move_base_client->isServerConnected()){
    ROS_INFO("Sending goal to move_base");
    move_base_client->sendGoal(goal);
    move_base_client->waitForResult();
    if(move_base_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("The base moved successfully!");
    else
      ROS_WARN("The base failed to move for some reason");
  }

  return true;*/
}

bool ArmPlanner::grasping_pipeline(){
  if(!open_gripper()) return false;
  sleep(.5);
  if(approaching_procedure(.4,.04*velocity_scale)){
    sleep(.5);
    if(grasping_procedure(.2,.02*velocity_scale)){
      sleep(.5);
      if(grasp()){
        sleep(.5);
        //move_away_from_targetUP(.2,.04);
        move_away_from_target(.2,.03*velocity_scale); return true;
       /* fixed_poses[7](4)=planner->current_joint_angles(4);
        if(move_to_fixed_pose(fixed_poses[7],.4,.04)){
          return !grasping_failure();
        }else{
          fixed_poses[8](4)=planner->current_joint_angles(4);
          if(move_to_fixed_pose(fixed_poses[8],.4,.04))
            return !grasping_failure();
        }*/
      }
    }
  }
  else{
    move_base_procedure();
    return false;
  }
  return false;
}

bool ArmPlanner::dropping_pipeline(){
  if(approaching_procedure(.4,.04*velocity_scale)){
    sleep(.5);
    if(grasping_procedure(.2,.02*velocity_scale))
    {
      sleep(.5);
      if(drop()){
        sleep(.5);
        move_away_from_target(.2,.04*velocity_scale);
        fixed_poses[7](4)=joint_offsets[4];
        if(move_to_fixed_pose(fixed_poses[7],.4,.04*velocity_scale)){
          return true;
        }else{
          fixed_poses[8](4)=joint_offsets[4];
          if(move_to_fixed_pose(fixed_poses[8],.4,.04*velocity_scale))
            return true;
        }
      }
    }
  }
  else{
    move_base_procedure();
    return false;
  }
  return false;
}

bool ArmPlanner::move_to_fixed_pose_cb(int pose_index)
{
  bool ris;
  replan=true;
  sleep(.5);
  set_planner_current_joint_state();
  stop_planning=false;
  int i=0;
  ris=move_to_fixed_pose(fixed_poses[pose_index],.4,.04*velocity_scale);
  return ris;
}

bool ArmPlanner::simple_target_cb()
{
  bool ris;
  replan=true;
  //sleep(.5);
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////777
  set_planner_current_joint_state();
 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////777
  stop_planning=false;
  int i=0;
  ris=simple_target_procedure(.4,.04*velocity_scale);
  if(!ris) move_base_procedure();
  return ris;
}

bool ArmPlanner::grasp_cb()
{
  if(!compute_approach_target_preferred_pitch(target_cartesian_position, approach_target, preferred_gripper_pitch)){
    ROS_WARN("approach target not found!");
    move_base_procedure();
    return false;
  }

  grasping_target=target_cartesian_position;
  replan=true;
  sleep(.5);
  set_planner_current_joint_state();
  stop_planning=false;
  bool ris=grasping_pipeline();
  return ris;
}

bool ArmPlanner::drop_cb()
{
  if(!compute_approach_target_preferred_pitch(target_cartesian_position, approach_target, preferred_gripper_pitch)){
    ROS_WARN("approach target not found!");
    move_base_procedure();
    return false;
  }
  
  grasping_target=target_cartesian_position;
  replan=true;
  sleep(.5);
  set_planner_current_joint_state();
  stop_planning=false;
  bool ris=dropping_pipeline();
  return ris;
}

bool ArmPlanner::check_IK_feasibility_cb(arm_planner::check_IK_feasibility::Request &req, arm_planner::check_IK_feasibility::Response &res)
{
  Eigen::Vector3d target_p, target_norm;
  target_p=Eigen::Vector3d(req.target_position[0], req.target_position[1], req.target_position[2]);
  target_norm=Eigen::Vector3d(req.target_normal[0], req.target_normal[1], req.target_normal[2]);
  res.feasible=planner->compute_IK(target_p, target_norm);
  
  if(!res.feasible){
    return true;
  }

  std::vector<double> jangles_reversed;
  jangles_reversed.push_back(planner->IK_srv.response.joint_angles[4]);
  jangles_reversed.push_back(planner->IK_srv.response.joint_angles[3]);
  jangles_reversed.push_back(planner->IK_srv.response.joint_angles[2]);
  jangles_reversed.push_back(planner->IK_srv.response.joint_angles[1]);
  jangles_reversed.push_back(planner->IK_srv.response.joint_angles[0]);
  if(planner->in_collision(jangles_reversed)){
    res.feasible= false;
    return true;
  }
  
  Eigen::Matrix<double, 5,1> j_pos;
  j_pos(0)=jangles_reversed[4]; j_pos(1)=jangles_reversed[3]; j_pos(2)=jangles_reversed[2]; j_pos(3)=jangles_reversed[1]; j_pos(4)=jangles_reversed[0];
  Eigen::Vector3d pos;
  compute_ee_position(j_pos, pos);
  res.ee_position[0]=pos(0); res.ee_position[1]=pos(1); res.ee_position[2]=pos(2);
  res.feasible=true;
  res.joints_target=planner->IK_srv.response.joint_angles;
  return true;
}

bool ArmPlanner::get_ee_pose_cb(arm_planner::get_ee_pose::Request &req, arm_planner::get_ee_pose::Response &res)
{
  Eigen::Matrix<double, 5,1> j_pos;
  j_pos(0)=req.joint_angles[0]; j_pos(1)=req.joint_angles[1]; j_pos(2)=req.joint_angles[2]; j_pos(3)=req.joint_angles[3]; j_pos(4)=req.joint_angles[4];
  Eigen::Affine3d ee_pose;
  compute_ee_pose(j_pos, ee_pose);
  int i=0;
  for(int r=0; r<4; r++){
    for(int c=0; c<4; c++){
      res.ee_pose[i]=ee_pose.matrix()(r,c);
      i++;
    }
  }
  
 /* //// debug ////
  i=0;
  for(i=0; i<res.ee_pose.size(); i++){
      std::cout<<res.ee_pose[i]<<" ";
      if((i+1)%4==0&&i!=0) std::cout<<std::endl;
  }
  std::cout<<std::endl;
  ///////////////*/
  return true;
}

bool ArmPlanner::open_gripper_cb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  bool ris=open_gripper();
  return ris;
}

bool ArmPlanner::close_gripper_cb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  bool ris=close_gripper();
  return ris;
}

void ArmPlanner::compute_action(const arm_planner::arm_planningGoalConstPtr & goal){
  stop_planning=true;
  result.move_base=false;
  Eigen::Matrix<double,5,1> cjs;
  get_current_joint_state(cjs);
  rosGoal2TargetRequest(goal, kinect_T, rgb_T, cjs, target_cartesian_position, target_cartesian_normal, preferred_gripper_pitch, gripper_roll, req_mode);
  
  if(req_mode==1){
    if(grasp_cb()) planningAction.setSucceeded(result);
    else planningAction.setAborted(result);
    return;
  }
  else if(req_mode==2){
    if(drop_cb()) planningAction.setSucceeded(result);
    else planningAction.setAborted(result);
    return;
  }
  else if(req_mode==0){
    if(simple_target_cb()) planningAction.setSucceeded(result);
    else planningAction.setAborted(result);
    return;
  }
  else if(req_mode==10){
    bool ris;
    replan=true;
    sleep(.5);
    set_planner_current_joint_state();
    stop_planning=false;
    Eigen::Matrix<double,5,1> joints_target;
    for(int i=0; i<5; i++)
      joints_target(i)=goal->joints_target[i];
      
    ris=move_to_fixed_pose(joints_target,.2,.04*velocity_scale);
    if(ris) planningAction.setSucceeded(result);
    else planningAction.setAborted(result);
    return;
  }
  else if(req_mode==-1){
    double yaw=yawFromTarget(target_cartesian_position);
    preferred_gripper_pitch-=0;
    Eigen::Vector3d rpy(gripper_roll*M_PI/180.0, preferred_gripper_pitch*M_PI/180, yaw);
    rpyToNorm(rpy,target_cartesian_normal);
    //yaw=2.94961-yaw;
    //double d=.30;
    double d=goal->gaze_distance;
    Eigen::Vector3d target;
    Eigen::Vector3d cam_offset(-0.0827602, -0.0941455, -0.0);
    Eigen::Vector3d target2;
    target2(2)=-target2(1)*cos(preferred_gripper_pitch*M_PI/180.0);
    target2(1)=-target2(0)*sin(yaw);
    target2(0)=-target2(0)*cos(yaw);
    
    
    target(0)=d*cos(yaw-M_PI)*cos(preferred_gripper_pitch*M_PI/180.0);
    target(1)=d*sin(yaw-M_PI)*cos(preferred_gripper_pitch*M_PI/180.0);
    target(2)=d*sin(preferred_gripper_pitch*M_PI/180.0);
    target_cartesian_position+=(target+target2); target_cartesian_position(2)+=.02;
    gripper_roll=90;
    if(simple_target_cb()) planningAction.setSucceeded(result);
    else planningAction.setAborted(result);
    return;
  }
  else if(req_mode<-1){
    if(move_to_fixed_pose_cb(-req_mode-1)) planningAction.setSucceeded(result);
    else planningAction.setAborted(result);
    return;
  }
  planningAction.setAborted(result);
}

} //namespace arm_planner


using namespace arm_planner;

int main(int argc, char **argv){
  
  ros::init(argc, argv, "planner_main");
    ros::NodeHandle nh;
  ros::AsyncSpinner spinner(8);
  ArmPlanner* arm_pl=new ArmPlanner(nh, ros::this_node::getName());

  // arm_pl->finger_position_client = nh.serviceClient<youbot_driver_ros_interface::GripperPositions>("get_gripper_positions");
  
  //arm_pl->pub_js = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);

  arm_pl->arm_pub_pos = nh.advertise<brics_actuator::JointPositions>("/arm_1/arm_controller/position_command", 1);
  arm_pl->gripper_pub_pos = nh.advertise<brics_actuator::JointPositions>("arm_1/gripper_controller/position_command", 1);
  arm_pl->planning_scene_pub = nh.advertise<arm_planner::PlanningSceneFrame>("/planning_scene_frame", 1);
  //ros::Subscriber target_request_sub = nh.subscribe("/target_request", 1, &ArmPlanner::target_request_cb, &*arm_pl);
  ros::Subscriber obstacles_sub = nh.subscribe("/obstacles_position", 1, &ArmPlanner::obstacles_position_cb, &*arm_pl);
  ros::Subscriber joint_state_sub = nh.subscribe("/joint_states", 1, &ArmPlanner::jointstateCallback, &*arm_pl);

  ros::ServiceServer srv_open_gripper=nh.advertiseService("arm_planner/open_gripper", &ArmPlanner::open_gripper_cb, &*arm_pl);
  ros::ServiceServer srv_close_gripper=nh.advertiseService("arm_planner/close_gripper", &ArmPlanner::close_gripper_cb, &*arm_pl);
  
  ros::ServiceServer srv_check_IK_feasibility=nh.advertiseService("arm_planner/check_IK_feasibility", &ArmPlanner::check_IK_feasibility_cb, &*arm_pl);
  ros::ServiceServer srv_ee_pose=nh.advertiseService("arm_planner/get_ee_pose", &ArmPlanner::get_ee_pose_cb, &*arm_pl);
  
  arm_pl->initPlanning(argv[1]);
  
  arm_pl->planner=new Planner(nh);
  arm_pl->control_points=arm_pl->planner->control_points;
  arm_pl->scene_control_points=arm_pl->control_points;
  arm_pl->real_control_points=arm_pl->control_points;
  
  spinner.start();
  sleep(2);

  arm_pl->jointState2jointPositions(arm_pl->current_jstate, arm_pl->start_pos);  
  //////
  Eigen::Matrix<double, 5,1> m;m<<0,0,0,0,0;
  arm_pl->planner->current_joint_angles=m;
  /////    
  arm_pl->set_planner_current_joint_state();
  

  while(nh.ok()){

    //////// VISUALIZATION ////////////////
    if (arm_pl->visualization){
      compute_manipulation_area(arm_pl->workspace,arm_pl->target_cartesian_position,arm_pl->gripper_pitch,0,arm_pl->manipulation_area);
      Eigen::Matrix<double,5,1> cjs;
      arm_pl->get_current_joint_state(cjs);
      compute_control_points_pos(arm_pl->real_control_points,cjs);
      compute_control_points_pos(arm_pl->scene_control_points,arm_pl->planner->current_joint_angles);
      arm_pl->scene_obstacles.clear();
      arm_pl->scene_obstacles=arm_pl->planner->fixed_obstacles;
      arm_pl->appendObstacles(arm_pl->obstacles, arm_pl->scene_obstacles);
      arm_planner::PlanningSceneFrame psf_msg;
      Eigen::Matrix3d ee_orient;
      compute_ee_orientation(arm_pl->planner->current_joint_angles, ee_orient);
      planningSceneFrame2RosMsg(arm_pl->real_control_points,arm_pl->scene_control_points, arm_pl->scene_obstacles, arm_pl->planner->ee_traj, arm_pl->target_cartesian_position, arm_pl->manipulation_area, ee_orient, psf_msg);
      arm_pl->planning_scene_pub.publish(psf_msg);
      ros::Rate rate(20);
      rate.sleep();

      
    }else{
      sleep(3);
    }
    ////////////////////////////////////////

  }

  return 1;
}


