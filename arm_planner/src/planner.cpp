/*
 * planner.cpp
 *
 *      Author: Marco Imperoli
 */


#include "arm_planner/planner.h"

/*#define l2 0.155
#define l3 0.135
#define l4 0.218
#define lox 0.033
#define loz 0.1472
#define lax 0*/

namespace arm_planner{

using namespace Eigen;
using namespace std;

Planner::Planner(ros::NodeHandle& nh){
  nh.param("youBotDriverCycleFrequencyInHz", driver_loop_rate, 50.0);
  nh.param("trjectoryGeneratorLoopRate", generator_loop_rate, 10.0);
  traj_growing_factor=round(driver_loop_rate/generator_loop_rate);
  js2cs_client=nh.serviceClient<trajectory_generator::JStoCS>("From_JS_to_CS");
  cs2cs_client=nh.serviceClient<trajectory_generator::CStoCS>("From_CS_to_CS");
  js2js_client=nh.serviceClient<trajectory_generator::JStoJS>("From_JS_to_JS");
  solve_ik_client = nh.serviceClient<ik_solver_service::SolveClosestIK>("solve_closest_ik");
  
  init_arm_joints();
  init_control_points();
  init_fixed_obstacles();
  init_trajectory_service();
  
  rand_range=200; rand_offset=100;

  help_routine=false;
}

Planner::~Planner(){}


bool Planner::plan(){
  
  temp_traj.points.clear();
  bool goal_reached=false;
  bool perturbation=false;
  int iter=0;
  if(!help_routine)
    planned_traj.points.clear();
  perturbations.clear();
  std::cout<<"start planning"<<std::endl;
  //ros::Rate lr(loop_rate);

  if(!compute_IK(target_cartesian_position, target_cartesian_normal)){
    feasible=false;
    std::cout<<"end planning (IK error)"<<std::endl;
    return false;
  }
  start_pos.positions[0].value=current_joint_angles(0);
  start_pos.positions[1].value=current_joint_angles(1);
  start_pos.positions[2].value=current_joint_angles(2);
  start_pos.positions[3].value=current_joint_angles(3);
  start_pos.positions[4].value=current_joint_angles(4);

  std::vector<double> jangles_reversed_2;
  jangles_reversed_2.push_back(current_joint_angles(4));
  jangles_reversed_2.push_back(current_joint_angles(3));
  jangles_reversed_2.push_back(current_joint_angles(2));
  jangles_reversed_2.push_back(current_joint_angles(1));
  jangles_reversed_2.push_back(current_joint_angles(0));
  if(in_collision(jangles_reversed_2)){
    std::cout<<"robot collision in initial position"<<std::endl;
    return false;
  }

  target_pos.positions[0].value=IK_srv.response.joint_angles[0];
  target_pos.positions[1].value=IK_srv.response.joint_angles[1];
  target_pos.positions[2].value=IK_srv.response.joint_angles[2];
  target_pos.positions[3].value=IK_srv.response.joint_angles[3];
  target_pos.positions[4].value=IK_srv.response.joint_angles[4];

  std::vector<double> jangles_reversed;
  jangles_reversed.push_back(IK_srv.response.joint_angles[4]);
  jangles_reversed.push_back(IK_srv.response.joint_angles[3]);
  jangles_reversed.push_back(IK_srv.response.joint_angles[2]);
  jangles_reversed.push_back(IK_srv.response.joint_angles[1]);
  jangles_reversed.push_back(IK_srv.response.joint_angles[0]);
  if(in_collision(jangles_reversed)){
    std::cout<<"robot collision in target position"<<std::endl;
    return false;
  }
  
  feasible=true;
  js2js.request.start_pos = start_pos;
  js2js.request.end_pos = target_pos;
  js2js.request.start_vel = start_vel;
  js2js.request.end_vel = target_vel;
  js2js.request.max_vel = max_vel;
  js2js.request.max_acc = max_acc;

  // rand_range=200;  rand_offset=100;
  while(iter<100&&!goal_reached){
    if (js2js_client.call(js2js))
    {
      if (js2js.response.feasible)
      {
        cout << "feasible" << endl;
        temp_traj = js2js.response.trajectory;
        planned_traj.header=temp_traj.header;
        planned_traj.joint_names=temp_traj.joint_names;
        //planned_traj.points.clear();

        for(int i=temp_traj.points.size()-1;i>=0;i--){
          if(in_collision(temp_traj.points[i].positions)){
            std::cout<<"X";
            perturbation=true;
            compute_perturbation_ee();
            break;
          }else{
            planned_traj.points.push_back(temp_traj.points[i]);
            std::cout<<".";
            if(i==0) {
              if(!perturbation){ goal_reached=true; std::cout<<"GOAL!"<<std::endl;}
              else {
                perturbation=false;
                js2js.request.start_pos=js2js.request.end_pos;
                js2js.request.start_vel = js2js.request.end_vel;
                js2js.request.end_pos = target_pos;
                js2js.request.end_vel = target_vel;

                perturbations.push_back(temp_traj.points[i]);
              }
            }
          }
        }
        std::cout<<std::endl;
      }
      else
      {
        cout << "NOT feasible" << endl;
        //compute_perturbation_ee(planned_traj);
        //feasible=false;
      }
    }
    else
    {
      ROS_ERROR("Could not call service.");
      //compute_perturbation_ee(planned_traj);
      //feasible=false;
      //break;
    }
    iter++;
  //  rand_range+=2; rand_offset+=1;
  }
  //rand_range=200; rand_offset=100;
  if(!goal_reached) return false;

  bool smoothed=false;
  if(perturbations.size()>0){
    std::cout<<"smoothing trajectory"<<std::endl;
    smoothed=smooth_trajectory();
    std::cout<<"smoothed? "<<smoothed<<std::endl;
    if(!smoothed) return false;
  }
  if(!help_routine){
    grows_trajectory(traj_growing_factor,planned_traj);
    compute_ee_trajectory(planned_traj);  
  }  
/*
  //// help routine //////////////////////////////////////////
  if((!goal_reached||(!smoothed&&perturbations.size()>0))&&!help_routine){
  //if(!goal_reached&&!help_routine){
    help_routine=true;
    planned_traj.points.clear();
    Eigen::Vector3d temp_target_cartesian_position(target_cartesian_position);
    Eigen::Vector3d temp_target_cartesian_normal(target_cartesian_normal);
    target_cartesian_position=Eigen::Vector3d(0,0,.65); target_cartesian_normal=Eigen::Vector3d(0,0,0);
    if(plan()){
      current_joint_angles(0)=target_pos.positions[0].value;
      current_joint_angles(1)=target_pos.positions[1].value;
      current_joint_angles(2)=target_pos.positions[2].value;
      current_joint_angles(3)=target_pos.positions[3].value;
      current_joint_angles(4)=target_pos.positions[4].value;
      target_cartesian_position=temp_target_cartesian_position;
      target_cartesian_normal=temp_target_cartesian_normal;
      if(plan()){
        std::cout<<"end planning (help)"<<std::endl;
        temp_traj.points.clear();
        help_routine=false;
        temp_traj.points.clear();
        grows_trajectory(traj_growing_factor,planned_traj);
        compute_ee_trajectory();
        return true;
      }
    }
    std::cout<<"end planning"<<std::endl;
    help_routine=false;
    temp_traj.points.clear();
    return false;
  }
  ////////////////////////////////////////////////////////////
*/
  std::cout<<"end planning"<<std::endl;
  temp_traj.points.clear();
  
  return true;
}

bool Planner::plan_fixed_pose(Eigen::Matrix<double,5,1>& t_pos){
  
  temp_traj.points.clear();
  bool goal_reached=false;
  bool perturbation=false;
  int iter=0;
  planned_traj.points.clear();
  perturbations.clear();
  std::cout<<"start planning"<<std::endl;
  //ros::Rate lr(loop_rate);

  start_pos.positions[0].value=current_joint_angles(0);
  start_pos.positions[1].value=current_joint_angles(1);
  start_pos.positions[2].value=current_joint_angles(2);
  start_pos.positions[3].value=current_joint_angles(3);
  start_pos.positions[4].value=current_joint_angles(4);

  std::vector<double> jangles_reversed_2;
  jangles_reversed_2.push_back(current_joint_angles(4));
  jangles_reversed_2.push_back(current_joint_angles(3));
  jangles_reversed_2.push_back(current_joint_angles(2));
  jangles_reversed_2.push_back(current_joint_angles(1));
  jangles_reversed_2.push_back(current_joint_angles(0));
  if(in_collision(jangles_reversed_2)){
    std::cout<<"robot collision in initial position"<<std::endl;
    return false;
  }

  target_pos.positions[0].value=t_pos(0);
  target_pos.positions[1].value=t_pos(1);
  target_pos.positions[2].value=t_pos(2);
  target_pos.positions[3].value=t_pos(3);
  target_pos.positions[4].value=t_pos(4);

  std::vector<double> jangles_reversed;
  jangles_reversed.push_back(t_pos(4));
  jangles_reversed.push_back(t_pos(3));
  jangles_reversed.push_back(t_pos(2));
  jangles_reversed.push_back(t_pos(1));
  jangles_reversed.push_back(t_pos(0));
  if(in_collision(jangles_reversed)){
    std::cout<<"robot collision in target position"<<std::endl;
    return false;
  }
  
  feasible=true;
  js2js.request.start_pos = start_pos;
  js2js.request.end_pos = target_pos;
  js2js.request.start_vel = start_vel;
  js2js.request.end_vel = target_vel;
  js2js.request.max_vel = max_vel;
  js2js.request.max_acc = max_acc;

  // rand_range=200;  rand_offset=100;
  while(iter<100&&!goal_reached){
    if (js2js_client.call(js2js))
    {
      if (js2js.response.feasible)
      {
        cout << "feasible" << endl;
        temp_traj = js2js.response.trajectory;
        planned_traj.header=temp_traj.header;
        planned_traj.joint_names=temp_traj.joint_names;
        //planned_traj.points.clear();

        for(int i=temp_traj.points.size()-1;i>=0;i--){
          if(in_collision(temp_traj.points[i].positions)){
            std::cout<<"X";
            perturbation=true;
            compute_perturbation_ee();
             break;
          }else{
            planned_traj.points.push_back(temp_traj.points[i]);
            std::cout<<".";
            if(i==0) {
              if(!perturbation){ goal_reached=true; std::cout<<"GOAL!"<<std::endl;}
              else {
                perturbation=false;
                js2js.request.start_pos=js2js.request.end_pos;
                js2js.request.start_vel = js2js.request.end_vel;
                js2js.request.end_pos = target_pos;
                js2js.request.end_vel = target_vel;

                perturbations.push_back(temp_traj.points[i]);
              }
            }
          }
        }
        std::cout<<std::endl;
      }
      else
      {
        cout << "NOT feasible" << endl;
        //compute_perturbation_ee(planned_traj);
        //feasible=false;
      }
    }
    else
    {
      ROS_ERROR("Could not call service. (fixed pose)");
      //compute_perturbation_ee(planned_traj);
      //feasible=false;
      //break;
    }
    iter++;
  //  rand_range+=2; rand_offset+=1;
  }
  //rand_range=200; rand_offset=100;
  if(!goal_reached) return false;

  bool smoothed=false;
  if(perturbations.size()>0){
    std::cout<<"smoothing trajectory"<<std::endl;
    smoothed=smooth_trajectory();
    std::cout<<"smoothed? "<<smoothed<<std::endl;
    if(!smoothed) return false;
  }
  if(!help_routine){
    grows_trajectory(traj_growing_factor,planned_traj);
    compute_ee_trajectory(planned_traj);  
  }  

  std::cout<<"end planning"<<std::endl;
  temp_traj.points.clear();
  
  return true;
}


bool Planner::plan_straight_traj(Eigen::Vector3d& st_pos, Eigen::Vector3d& t_pos, double pitch, double roll,trajectory_msgs::JointTrajectory& pl_traj){
  pl_traj.points.clear();
  Eigen::Vector3d rpy((roll*M_PI/180.0)/*+joint_offsets[4]*/, pitch*M_PI/180, yawFromTarget(t_pos));

  //Eigen::Vector3d normal(.7071,.7071,0);
  /*Eigen::Vector3d normal;
  Eigen::Vector3d rpy(roll*M_PI/180.0, pitch*M_PI/180, yawFromTarget(st_pos));
  rpyToNorm(rpy,normal);
  if(!compute_IK(st_pos, normal)){
    std::cout<<"end planning (IK error straight_traj)"<<std::endl;
    return false;
  }
  start_pos.positions[0].value=IK_srv.response.joint_angles[0];
  start_pos.positions[1].value=IK_srv.response.joint_angles[1];
  start_pos.positions[2].value=IK_srv.response.joint_angles[2];
  start_pos.positions[3].value=IK_srv.response.joint_angles[3];
  start_pos.positions[4].value=IK_srv.response.joint_angles[4];*/
  

  geometry_msgs::Pose end_p;
  Matrix3d m;
  /*m=AngleAxisd(rpy(2), Vector3d::UnitZ())
  *AngleAxisd(rpy(1), Vector3d::UnitY());*/
  Matrix3d Rx; Rx<<1,0,0,0,cos(rpy(0)), -sin(rpy(0)),0, sin(rpy(0)), cos(rpy(0));
    Matrix3d Rz; Rz<<cos(rpy(2)), -sin(rpy(2)),0, sin(rpy(2)), cos(rpy(2)),0, 0,0,1;
    Matrix3d Ry; Ry<<cos(rpy(1)),0, sin(rpy(1)),0, 1,0, -sin(rpy(1)),0,cos(rpy(1));
    m=Rz*Ry*Rx;
  
  //m = AngleAxisd(rpy(2), Vector3d::UnitZ())
  //* AngleAxisd(rpy(1), Vector3d::UnitY())
  // * AngleAxisd(rpy(0), Vector3d::UnitX());
  //* AngleAxisd(/*rpy(0)*/M_PI/2, Vector3d::UnitX());
  //  * AngleAxisd(/*rpy(0)*/-M_PI/2, Vector3d::UnitX());
  //* AngleAxisd(-M_PI, Vector3d::UnitX());

  Eigen::Quaterniond grip(m);  
  end_p.position.x = t_pos(0);
  end_p.position.y = t_pos(1);
  end_p.position.z = t_pos(2);
  //if(grip.w()>0){
    end_p.orientation.x = grip.x();
    end_p.orientation.y = grip.y();
    end_p.orientation.z = grip.z();
    end_p.orientation.w = grip.w();
  /*}else{
    end_p.orientation.x = -grip.x();
    end_p.orientation.y = -grip.y();
    end_p.orientation.z = -grip.z();
    end_p.orientation.w = -grip.w();
  }*/

  geometry_msgs::Pose st_p;
  st_p.position.x = st_pos(0);
  st_p.position.y = st_pos(1);
  st_p.position.z = st_pos(2);
  st_p.orientation=end_p.orientation;

  cs2cs.request.start_pos = st_p;
  cs2cs.request.end_pos = end_p;
  cs2cs.request.start_vel = 0;
  cs2cs.request.end_vel = 0;
  cs2cs.request.max_vel = max_vel;
  cs2cs.request.max_acc = max_acc;

  /*js2cs.request.start_pos = start_pos;
  js2cs.request.end_pos = end_p;
  js2cs.request.start_vel = 0;
  js2cs.request.end_vel = 0;
  js2cs.request.max_vel = max_vel;
  js2cs.request.max_acc = max_acc;*/

  if (cs2cs_client.call(cs2cs)){
    if (cs2cs.response.feasible){
      for(int i=cs2cs.response.trajectory.points.size()-1;i>=0;i--){
        if(in_collision(cs2cs.response.trajectory.points[i].positions)){
          return false;
        }
      }
    }else{
      ROS_WARN("straight_line not feasible");
          return false;
      /*if(theta>0) theta-=180;
      else theta+=180;
      m = AngleAxisd(theta*M_PI/180.0, Vector3d::UnitZ())
      * AngleAxisd(0.5*M_PI, Vector3d::UnitY());
      Eigen::Quaterniond grip(m);  ;
      end_p.orientation.x = grip.x();
      end_p.orientation.y = grip.y();
      end_p.orientation.z = grip.z();
      end_p.orientation.w = grip.w();
      js2cs.request.end_pos = end_p;

      if (js2cs_client.call(js2cs)){
        if (js2cs.response.feasible){
          for(int i=js2cs.response.trajectory.points.size()-1;i>=0;i--){
            if(in_collision(js2cs.response.trajectory.points[i].positions)){
              return false;
            }
          }
        }else{
          ROS_WARN("straight_line not feasible");
          return false;
        }
      }else{
        ROS_WARN("js2cs_client cannot be called!");
        return false;
      }*/

    }
  }else{
    ROS_WARN("js2cs_client cannot be called!");
    return false;
  }

  pl_traj=cs2cs.response.trajectory;
  flip_trajectory(pl_traj.points);
  grows_trajectory(traj_growing_factor,pl_traj);
  compute_ee_trajectory(pl_traj);
  return true;
}

bool Planner::plan_straight_traj(Eigen::Matrix<double, 5, 1>& st_joints, Eigen::Vector3d& t_pos, double pitch, double roll, trajectory_msgs::JointTrajectory& pl_traj){
  pl_traj.points.clear();

  Eigen::Vector3d st_pos;
  compute_ee_position(st_joints, st_pos);

  Eigen::Vector3d rpy((roll*M_PI/180)/*+joint_offsets[4]*/, pitch*M_PI/180, yawFromTarget(t_pos));

  /*start_pos.positions[0].value=st_joints(0);
  start_pos.positions[1].value=st_joints(1);
  start_pos.positions[2].value=st_joints(2);
  start_pos.positions[3].value=st_joints(3);
  start_pos.positions[4].value=st_joints(4);*/

  geometry_msgs::Pose end_p;

  Matrix3d m;
  /*m=AngleAxisd(rpy(2), Vector3d::UnitZ())
  *AngleAxisd(rpy(1), Vector3d::UnitY());*/
    Matrix3d Rx; Rx<<1,0,0,0,cos(rpy(0)), -sin(rpy(0)),0, sin(rpy(0)), cos(rpy(0));
    Matrix3d Rz; Rz<<cos(rpy(2)), -sin(rpy(2)),0, sin(rpy(2)), cos(rpy(2)),0, 0,0,1;
    Matrix3d Ry; Ry<<cos(rpy(1)),0, sin(rpy(1)),0, 1,0, -sin(rpy(1)),0,cos(rpy(1));
    m=Rz*Ry*Rx;
  //m = AngleAxisd(rpy(2), Vector3d::UnitZ())
  //* AngleAxisd(rpy(1), Vector3d::UnitY())
  // * AngleAxisd(rpy(0), Vector3d::UnitX());
  //* AngleAxisd(/*rpy(0)*/M_PI/2, Vector3d::UnitX());
  //  * AngleAxisd(/*rpy(0)*/-M_PI/2, Vector3d::UnitX());
  //* AngleAxisd(-M_PI, Vector3d::UnitX());

  end_p.position.x = t_pos(0);
  end_p.position.y = t_pos(1);
  end_p.position.z = t_pos(2);
  Eigen::Quaterniond grip(m);
  //if(grip.w()>0){
    end_p.orientation.x = grip.x();
    end_p.orientation.y = grip.y();
    end_p.orientation.z = grip.z();
    end_p.orientation.w = grip.w();
  /*}else{
    end_p.orientation.x = -grip.x();
    end_p.orientation.y = -grip.y();
    end_p.orientation.z = -grip.z();
    end_p.orientation.w = -grip.w();
  }*/

  geometry_msgs::Pose st_p;
  st_p.position.x = st_pos(0);
  st_p.position.y = st_pos(1);
  st_p.position.z = st_pos(2);
  st_p.orientation=end_p.orientation;

  cs2cs.request.start_pos = st_p;
  cs2cs.request.end_pos = end_p;
  cs2cs.request.start_vel = 0;
  cs2cs.request.end_vel = 0;
  cs2cs.request.max_vel = max_vel;
  cs2cs.request.max_acc = max_acc;


  if (cs2cs_client.call(cs2cs)){
    if (cs2cs.response.feasible){
      for(int i=cs2cs.response.trajectory.points.size()-1;i>=0;i--){
        if(in_collision(cs2cs.response.trajectory.points[i].positions)){
          return false;
        }
      }
    }else{
      ROS_WARN("straight_line not feasible");
          return false;
      /*if(theta>0) theta-=180;
      else theta+=180;
      m = AngleAxisd(theta*M_PI/180.0, Vector3d::UnitZ())
      * AngleAxisd(0.5*M_PI, Vector3d::UnitY());
      Eigen::Quaterniond grip(m);  ;
      end_p.orientation.x = grip.x();
      end_p.orientation.y = grip.y();
      end_p.orientation.z = grip.z();
      end_p.orientation.w = grip.w();
      js2cs.request.end_pos = end_p;

      if (js2cs_client.call(js2cs)){
        if (js2cs.response.feasible){
          for(int i=js2cs.response.trajectory.points.size()-1;i>=0;i--){
            if(in_collision(js2cs.response.trajectory.points[i].positions)){
              return false;
            }
          }
        }else{
          ROS_WARN("straight_line not feasible");
          return false;
        }
      }else{
        ROS_WARN("js2cs_client cannot be called!");
        return false;
      }*/
    }
  }else{
    ROS_WARN("js2cs_client cannot be called!");
    return false;
  }

  pl_traj=cs2cs.response.trajectory;
  flip_trajectory(pl_traj.points);
  grows_trajectory(traj_growing_factor,pl_traj);
  compute_ee_trajectory(pl_traj);
  return true;
}

void Planner::grows_trajectory(int factor, trajectory_msgs::JointTrajectory& traj){
  std::cout<<"before growing "<<traj.points.size()<<std::endl;
  trajectory_msgs::JointTrajectoryPoint p1,p2,temp_p;
  trajectory_msgs::JointTrajectory temp;

  if(traj.points.size()<2) return;
  p1=traj.points[0];
  temp.points.push_back(p1);
  for(int i=0;i<traj.points.size()-1;i++){
    p1=traj.points[i];
    p2=traj.points[i+1];
    //temp.points.push_back(p1);
    for (int j=1;j<factor;j++){
      temp_p.positions.clear();
      temp_p.velocities.clear();
      temp_p.accelerations.clear();
      temp_p.effort.clear();
      double delta=(double)j/(double)factor;
      for(int k=0;k<5;k++){
        temp_p.positions.push_back(p1.positions[k]+(p2.positions[k]-p1.positions[k])*delta);
        temp_p.velocities.push_back(p1.velocities[k]+(p2.velocities[k]-p1.velocities[k])*delta);
        temp_p.accelerations.push_back(p1.accelerations[k]+(p2.accelerations[k]-p1.accelerations[k])*delta);
        //temp_p.effort.push_back(p1.effort[k]+(p2.effort[k]-p1.effort[k])*delta);
      }
      temp_p.time_from_start=p1.time_from_start;
      temp.points.push_back(temp_p);
    }
    temp.points.push_back(p2);
  }
  traj.points=temp.points;
  std::cout<<"after growing "<<traj.points.size()<<std::endl;
}

bool Planner::smooth_trajectory(){
  smoothed_traj.points.clear();
  trajectory_msgs::JointTrajectoryPoint target=planned_traj.points.back();
  trajectory_msgs::JointTrajectoryPoint begin=planned_traj.points.front();
  
  smooth_trajectory_routine(target, perturbations.back());
  for(int i=perturbations.size()-1;i>=0;){
    int aux=i;
    for (int j=-1;j<i;j++){
      if(j==-1){
        if(smooth_trajectory_routine(perturbations[i], begin)){
          flip_trajectory(smoothed_traj.points);
          std::cout<<"size smoothed "<<smoothed_traj.points.size()<<std::endl;
          planned_traj=smoothed_traj;
          return true;
        }
      }else{
        if(smooth_trajectory_routine(perturbations[i], perturbations[j])){
          i=j;
        }
      }
    }
    if (aux==i) return false;
  }
  return false;
}

bool Planner::smooth_trajectory_routine(trajectory_msgs::JointTrajectoryPoint& start, trajectory_msgs::JointTrajectoryPoint& end){

  Eigen::VectorXd pos_start(5);
  pos_start(0)=start.positions[4]; pos_start(1)=start.positions[3]; pos_start(2)=start.positions[2]; pos_start(3)=start.positions[1]; pos_start(4)=start.positions[0];
  Eigen::VectorXd pos_end(5);
  pos_end(0)=end.positions[4]; pos_end(1)=end.positions[3]; pos_end(2)=end.positions[2]; pos_end(3)=end.positions[1]; pos_end(4)=end.positions[0]; 
  //pos=youbot2matlab(pos);
  js2js.request.start_pos.positions[0].value=pos_start(0);
  js2js.request.start_pos.positions[1].value=pos_start(1);
  js2js.request.start_pos.positions[2].value=pos_start(2);
  js2js.request.start_pos.positions[3].value=pos_start(3);
  js2js.request.start_pos.positions[4].value=pos_start(4);

  js2js.request.start_vel.velocities[0].value=0;//last.velocities[0];
  js2js.request.start_vel.velocities[1].value=0;//last.velocities[1];
  js2js.request.start_vel.velocities[2].value=0;//last.velocities[2];
  js2js.request.start_vel.velocities[3].value=0;//last.velocities[3];
  js2js.request.start_vel.velocities[4].value=0;//last.velocities[4];

  js2js.request.end_pos.positions[0].value=pos_end(0);
  js2js.request.end_pos.positions[1].value=pos_end(1);
  js2js.request.end_pos.positions[2].value=pos_end(2);
  js2js.request.end_pos.positions[3].value=pos_end(3);
  js2js.request.end_pos.positions[4].value=pos_end(4);

  js2js.request.end_vel.velocities=js2js.request.start_vel.velocities;

  temp_smoothed_traj.points.clear();

  if (js2js_client.call(js2js))
  {
    if (js2js.response.feasible)
    {
      temp_traj = js2js.response.trajectory;
      smoothed_traj.header=temp_traj.header;
      smoothed_traj.joint_names=temp_traj.joint_names;

      for(int i=temp_traj.points.size()-1;i>=0;i--){
        if(in_collision(temp_traj.points[i].positions)){
          std::cout<<"X";
          return false;
           break;
        }else{
          temp_smoothed_traj.points.push_back(temp_traj.points[i]);
          std::cout<<".";///////////////7
          if(i==0) {
            for(int j=0;j<temp_smoothed_traj.points.size();j++){
              smoothed_traj.points.push_back(temp_smoothed_traj.points[j]);
            }
            return true;
          }
        }
      }
      std::cout<<std::endl;
    }
    else
    {
      cout << "NOT feasible (smooth)" << endl;
      return false;
    }
  }
  else
  {
    ROS_ERROR("Could not call service. (smooth)");
    return false;
  }
  return true;
}


void Planner::compute_ee_trajectory(trajectory_msgs::JointTrajectory& tr){
  std::cout<<"computing ee_traj"<<std::endl;
  ee_traj.clear();
  Eigen::Matrix<double, 5,1> j_pos;
  std::vector<double> joints_pos;
  for(int i=0;i<tr.points.size(); i++){
    joints_pos=tr.points[i].positions;
    j_pos(0)=joints_pos[4]; j_pos(1)=joints_pos[3]; j_pos(2)=joints_pos[2]; j_pos(3)=joints_pos[1]; j_pos(4)=joints_pos[0];
    j_pos=youbot2matlab(j_pos);
    forwardKin(j_pos, control_points[ee_index]);
    ee_traj.push_back(control_points[ee_index].position);
  }
}

void Planner::compute_control_points_position(std::vector<double>& joints_pos){
  Eigen::Matrix<double, 5,1> j_pos;
  j_pos(0)=joints_pos[4]; j_pos(1)=joints_pos[3]; j_pos(2)=joints_pos[2]; j_pos(3)=joints_pos[1]; j_pos(4)=joints_pos[0];
  j_pos=youbot2matlab(j_pos);

  for(int i=0;i<control_points.size();i++){
    if(i==cam_index){
      forwardKin(j_pos, control_points[i+3]);
      Eigen::Matrix3d ee_orient; forwardKinOrientationEE(j_pos, ee_orient);
      control_points[i].position=ee_orient*Eigen::Vector3d(-.175,-.07,0)+control_points[i+3].position;
      control_points[i+1].position=ee_orient*Eigen::Vector3d(-.125,-.07,0)+control_points[i+3].position;
      control_points[i+2].position=ee_orient*Eigen::Vector3d(-.225,-.07,0)+control_points[i+3].position;
      break;
    }
    if (control_points[i].link>0){
      forwardKin(j_pos, control_points[i]);
    }
  }
}

void Planner::compute_arm_joints_position(std::vector<double>& joints_pos){
  Eigen::Matrix<double, 5,1> j_pos;
  j_pos(0)=joints_pos[4]; j_pos(1)=joints_pos[3]; j_pos(2)=joints_pos[2]; j_pos(3)=joints_pos[1]; j_pos(4)=joints_pos[0];
  j_pos=youbot2matlab(j_pos);

  for(int i=0;i<arm_joints.size();i++){
    if (arm_joints[i].link>0){
      forwardKin(j_pos, arm_joints[i]);
    }
  }
}

void Planner::init_arm_joints(){
  ControlPoint cp1(1,loz,.031);
  ControlPoint cp2(2,l2,.031);
  ControlPoint cp3(3,l3,.031);
  ControlPoint cp4(4,l4,.031);

  arm_joints.push_back(cp1); arm_joints.push_back(cp2); arm_joints.push_back(cp3); arm_joints.push_back(cp4); 
}


void Planner::init_control_points(){
  // CONTROL POINTS MUST BE ORDERED!!!

  //ControlPoint cp01(-.3,0,0,.01);
  ControlPoint cp0(1,loz/2,loz/2);
  
  //control_points.push_back(cp01);
  ControlPoint cp1(2,0,.055);
  ControlPoint cp2b(2,l2-2*l2/3,.04);
  ControlPoint cp2a(2,l2-l2/3,.04);
  ControlPoint cp2(2,l2,.045);
  ControlPoint cp3b(3,l3-2*l3/3,.04);
  ControlPoint cp3a(3,l3-l3/3,.04);
  ControlPoint cp3(3,l3,.045);
  ControlPoint cp4b(4,.135/2,.045);
  ControlPoint cp4a(4,l4-l4/2,.05);
  ControlPoint cp4c(4,l4-.1,.05);
  ControlPoint cp4(4,l4-.05,.05);
  ControlPoint cp_cam(5,l4,.025);
  ControlPoint cp_cam2(5,l4,.025);
  ControlPoint cp_cam3(5,l4,.025);
  ControlPoint ee(4,l4,.0001);

  control_points.push_back(cp0);
  control_points.push_back(cp1);
  control_points.push_back(cp2b);
  control_points.push_back(cp2a);
  control_points.push_back(cp2);
  control_points.push_back(cp3b);
  control_points.push_back(cp3a);
  control_points.push_back(cp3); 
  control_points.push_back(cp4b);
  control_points.push_back(cp4a);
  control_points.push_back(cp4c);
  control_points.push_back(cp4);
  control_points.push_back(cp_cam);
  control_points.push_back(cp_cam2);
  control_points.push_back(cp_cam3);
  control_points.push_back(ee);//ee

  cam_index=12;
  ee_index=15;

  control_points_temp=control_points;
}

void Planner::init_trajectory_service(){
  brics_actuator::JointValue jv; jv.unit="rad";
  jv.joint_uri="arm_joint_1"; start_pos.positions.push_back(jv); target_pos.positions.push_back(jv); 
  jv.joint_uri="arm_joint_2"; start_pos.positions.push_back(jv); target_pos.positions.push_back(jv);
  jv.joint_uri="arm_joint_3"; start_pos.positions.push_back(jv); target_pos.positions.push_back(jv);
  jv.joint_uri="arm_joint_4"; start_pos.positions.push_back(jv); target_pos.positions.push_back(jv);
  jv.joint_uri="arm_joint_5"; start_pos.positions.push_back(jv); target_pos.positions.push_back(jv);

  brics_actuator::JointValue jv_vel; jv_vel.unit="s^-1 rad"; jv_vel.value=0;
  jv_vel.joint_uri="arm_joint_1"; start_vel.velocities.push_back(jv_vel); target_vel.velocities.push_back(jv_vel);
  jv_vel.joint_uri="arm_joint_2"; start_vel.velocities.push_back(jv_vel); target_vel.velocities.push_back(jv_vel);
  jv_vel.joint_uri="arm_joint_3"; start_vel.velocities.push_back(jv_vel); target_vel.velocities.push_back(jv_vel);
  jv_vel.joint_uri="arm_joint_4"; start_vel.velocities.push_back(jv_vel); target_vel.velocities.push_back(jv_vel);
  jv_vel.joint_uri="arm_joint_5"; start_vel.velocities.push_back(jv_vel); target_vel.velocities.push_back(jv_vel);
}

bool Planner::in_collision(std::vector<double>& joints_pos){
  compute_control_points_position(joints_pos);
  float eps=.01;

  // Fixed Obstacles Collisions
  for(int i=0;i<control_points.size();i++){
    if((control_points[i].position(2)-control_points[i].radius)<-0.11) return true;//ground collision
    for(int j=0;j<fixed_obstacles.size();j++){
      if(control_points[i].link>0){
        if(fixed_obstacles[j].radius>=0){//// sphere obst
          if((control_points[i].position-fixed_obstacles[j].position).norm()<(control_points[i].radius+fixed_obstacles[j].radius)) return true;
        }else{///box obst
          if((control_points[i].position(0)+control_points[i].radius)>fixed_obstacles[j].min_position(0)&&(control_points[i].position(1)+control_points[i].radius)>fixed_obstacles[j].min_position(1)&&(control_points[i].position(2)+control_points[i].radius)>fixed_obstacles[j].min_position(2)&&(control_points[i].position(0)-control_points[i].radius)<fixed_obstacles[j].max_position(0)&&(control_points[i].position(1)-control_points[i].radius)<fixed_obstacles[j].max_position(1)&&(control_points[i].position(2)-control_points[i].radius)<fixed_obstacles[j].max_position(2))
            return true;
        }
      }
    }
  }

  // Obstacles Collisions
  for(int i=0;i<control_points.size();i++){
    for(int j=0;j<obstacles.size();j++){
      if(control_points[i].link>0){
        if(obstacles[j].normal(0)!=0||obstacles[j].normal(1)!=0||obstacles[j].normal(2)!=0){//// plane obst
          Eigen::Vector3d diff=control_points[i].position-obstacles[j].position;
          if(diff.dot(obstacles[j].normal)<control_points[i].radius) return true;
        }else if(obstacles[j].radius>0){//// sphere obst
          if((control_points[i].position-obstacles[j].position).norm()<(control_points[i].radius+obstacles[j].radius+eps)) return true;
        }else{///box obst
          if((control_points[i].position(0)+control_points[i].radius)>(obstacles[j].min_position(0)-eps)&&(control_points[i].position(1)+control_points[i].radius)>(obstacles[j].min_position(1)-eps)&&(control_points[i].position(2)+control_points[i].radius)>(obstacles[j].min_position(2)-eps)&&(control_points[i].position(0)-control_points[i].radius)<(obstacles[j].max_position(0)+eps)&&(control_points[i].position(1)-control_points[i].radius)<(obstacles[j].max_position(1)+eps)&&(control_points[i].position(2)-control_points[i].radius)<(obstacles[j].max_position(2)+eps))
            return true;
        }
      }
    }
  }

  // Self Collisions
  // if control points are on the same or consecutive link --> they cannot collide
  for(int i=0;i<control_points.size();i++){
    for(int j=i+1;j<control_points.size();j++){
      if(control_points[i].link!=control_points[j].link&&abs(control_points[i].link-control_points[j].link)>1){
        if((control_points[i].position-control_points[j].position).norm()<(control_points[i].radius+control_points[j].radius)) return true;
      }
    }
  }
  return false;
}

void Planner::compute_perturbation(){
  //srand (time(0));
  int range=200; int offset=100;
  double pert0=(double)(rand()%range-offset)/10000.0;
  double pert1=(double)(rand()%range-offset)/10000.0;
  double pert2=(double)(rand()%range-offset)/10000.0;
  double pert3=(double)(rand()%range-offset)/10000.0;
  double pert4=(double)(rand()%range-offset)/10000.0;

  trajectory_msgs::JointTrajectoryPoint  last=planned_traj.points.back();

  Eigen::VectorXd pos(5);
  pos(0)=last.positions[4]; pos(1)=last.positions[3]; pos(2)=last.positions[2]; pos(3)=last.positions[1]; pos(4)=last.positions[0]; 
  //pos=youbot2matlab(pos);
  js2js.request.start_pos.positions[0].value=pos(0);
  js2js.request.start_pos.positions[1].value=pos(1);
  js2js.request.start_pos.positions[2].value=pos(2);
  js2js.request.start_pos.positions[3].value=pos(3);
  js2js.request.start_pos.positions[4].value=pos(4);

  js2js.request.start_vel.velocities[0].value=0;//last.velocities[0];
  js2js.request.start_vel.velocities[1].value=0;//last.velocities[1];
  js2js.request.start_vel.velocities[2].value=0;//last.velocities[2];
  js2js.request.start_vel.velocities[3].value=0;//last.velocities[3];
  js2js.request.start_vel.velocities[4].value=0;//last.velocities[4];

  js2js.request.end_pos.positions[0].value=js2js.request.start_pos.positions[0].value+10*pert0;
  js2js.request.end_pos.positions[1].value=js2js.request.start_pos.positions[1].value+10*pert1;
  js2js.request.end_pos.positions[2].value=js2js.request.start_pos.positions[2].value+10*pert2;
  js2js.request.end_pos.positions[3].value=js2js.request.start_pos.positions[3].value+10*pert3;
  js2js.request.end_pos.positions[4].value=js2js.request.start_pos.positions[4].value+10*pert4;

  js2js.request.end_vel.velocities=js2js.request.start_vel.velocities;

  //std::cout<<"\npert "<<pert0<<" "<<pert1<<" "<<pert2<<" "<<pert3<<" "<<pert4<<std::endl;

  //std::cout<<pos<<std::endl;

  
}

void Planner::compute_perturbation_ee(){
  //srand (time(0));
  bool flag=false;

  trajectory_msgs::JointTrajectoryPoint  last=planned_traj.points.back();
  compute_control_points_position(last.positions);

  while(!flag){
    double pert0=(double)(rand()%rand_range-rand_offset)/1000.0;
    double pert1=(double)(rand()%rand_range-rand_offset)/1000.0;
    double pert2=(double)(rand()%rand_range-rand_offset)/1000.0;

    Eigen::Vector3d ee_pos(control_points[ee_index].position(0),control_points[ee_index].position(1),control_points[ee_index].position(2));
    Eigen::Vector3d ee_pos_pert(ee_pos(0)+pert0,ee_pos(1)+pert1,ee_pos(2)+pert2);
    Eigen::Vector3d n(0,0,0);
    if(!compute_IK(ee_pos_pert, n)){
      std::cout<<"retry perturbation (IK error)"<<std::endl;
    }else{ flag=true;}
  }

  Eigen::VectorXd pos(5);
  pos(0)=last.positions[4]; pos(1)=last.positions[3]; pos(2)=last.positions[2]; pos(3)=last.positions[1]; pos(4)=last.positions[0]; 
  //pos=youbot2matlab(pos);
  js2js.request.start_pos.positions[0].value=pos(0);
  js2js.request.start_pos.positions[1].value=pos(1);
  js2js.request.start_pos.positions[2].value=pos(2);
  js2js.request.start_pos.positions[3].value=pos(3);
  js2js.request.start_pos.positions[4].value=pos(4);

  js2js.request.start_vel.velocities[0].value=0;//last.velocities[0];
  js2js.request.start_vel.velocities[1].value=0;//last.velocities[1];
  js2js.request.start_vel.velocities[2].value=0;//last.velocities[2];
  js2js.request.start_vel.velocities[3].value=0;//last.velocities[3];
  js2js.request.start_vel.velocities[4].value=0;//last.velocities[4];

  js2js.request.end_pos.positions[0].value=IK_srv.response.joint_angles[0];
  js2js.request.end_pos.positions[1].value=IK_srv.response.joint_angles[1];
  js2js.request.end_pos.positions[2].value=IK_srv.response.joint_angles[2];
  js2js.request.end_pos.positions[3].value=IK_srv.response.joint_angles[3];
  js2js.request.end_pos.positions[4].value=IK_srv.response.joint_angles[4];

  js2js.request.end_vel.velocities=js2js.request.start_vel.velocities;

  //std::cout<<"\npert "<<pert0<<" "<<pert1<<" "<<pert2<<" "<<pert3<<" "<<pert4<<std::endl;

  //std::cout<<pos<<std::endl;

  
}

bool Planner::compute_IK(Eigen::Vector3d& pos, Eigen::Vector3d& n, Eigen::Matrix<double, 5, 1> &out_joints){
  IK_srv.request.joint_angles[0]=out_joints(0);
  IK_srv.request.joint_angles[1]=out_joints(1);
  IK_srv.request.joint_angles[2]=out_joints(2);
  IK_srv.request.joint_angles[3]=out_joints(3);
  IK_srv.request.joint_angles[4]=out_joints(4);

  IK_srv.request.des_position[0]=pos(0);
  IK_srv.request.des_position[1]=pos(1);
  IK_srv.request.des_position[2]=pos(2);

  IK_srv.request.des_normal[0]=n(0);
  IK_srv.request.des_normal[1]=n(1);
  IK_srv.request.des_normal[2]=n(2);
  
  if (solve_ik_client.call(IK_srv))
  {
    if(!IK_srv.response.feasible)
    {
    //  ROS_WARN("ik not feasible");
      return false;
    }
  }
  else
  {
    //ROS_WARN("ik error!!");
    return false;
  }
  return true;
}

bool Planner::compute_IK(Eigen::Vector3d& pos, Eigen::Vector3d& n){
  IK_srv.request.joint_angles[0]=current_joint_angles(0);
  IK_srv.request.joint_angles[1]=current_joint_angles(1);
  IK_srv.request.joint_angles[2]=current_joint_angles(2);
  IK_srv.request.joint_angles[3]=current_joint_angles(3);
  IK_srv.request.joint_angles[4]=current_joint_angles(4);

  IK_srv.request.des_position[0]=pos(0);
  IK_srv.request.des_position[1]=pos(1);
  IK_srv.request.des_position[2]=pos(2);

  IK_srv.request.des_normal[0]=n(0);
  IK_srv.request.des_normal[1]=n(1);
  IK_srv.request.des_normal[2]=n(2);
  
  if (solve_ik_client.call(IK_srv))
  {
    if(!IK_srv.response.feasible)
    {
    //  ROS_WARN("ik not feasible");
      return false;
    }
  }
  else
  {
    ROS_WARN("ik error!!");
    return false;
  }
  return true;
}

void Planner::init_fixed_obstacles(){
  fixed_obstacles.clear();

  //base
  Obstacle o5(Eigen::Vector3d(-.43,-.160,-.1),Eigen::Vector3d(.115,.160,0));
  Obstacle o6(Eigen::Vector3d(-.43,-.20,.044),Eigen::Vector3d(-.1,.190,.047));
  Obstacle o7(.08,.14,-.055,.053);
  Obstacle o8(.08,-.14,-.055,.053);
  Obstacle o9(-.39,-.14,-.055,.053);
  Obstacle o10(-.39,.14,-.055,.053);
  // laser
  Obstacle o11(Eigen::Vector3d(.115,-.05,-.1),Eigen::Vector3d(.210,.05,.0));
  // asta kinect
  Obstacle o13(Eigen::Vector3d(-.42,-.02,.046),Eigen::Vector3d(-.38,.02,.65));
  // kinect
  Obstacle o14(Eigen::Vector3d(-.49,-.15,.52),Eigen::Vector3d(-.3,.15,.65));
  ///

  //Obstacle o12(Eigen::Vector3d(-.015,-.265,-.1),Eigen::Vector3d(.015,-.235,0.03));

  Obstacle o100(Eigen::Vector3d(.32,.10,-.05),Eigen::Vector3d(.6,.18,.45));
  Obstacle o101(Eigen::Vector3d(.05,-.8,-.05),Eigen::Vector3d(.12,-.3,.25));
  //Obstacle o102(Eigen::Vector3d(.35,-.2,.4),Eigen::Vector3d(.38,.2,.41));


  //obstacles.push_back(o1);
  //obstacles.push_back(o2);
  //obstacles.push_back(o3);
  //obstacles.push_back(o4);
  fixed_obstacles.push_back(o5);
  //fixed_obstacles.push_back(o6);
  fixed_obstacles.push_back(o7);
  fixed_obstacles.push_back(o8);
  fixed_obstacles.push_back(o9);
  fixed_obstacles.push_back(o10);
  fixed_obstacles.push_back(o11);
  //obstacles.push_back(o12);
  fixed_obstacles.push_back(o13);
  fixed_obstacles.push_back(o14);
  
  //fixed_obstacles.push_back(o100);
  //fixed_obstacles.push_back(o101);
  //fixed_obstacles.push_back(o102);
}



}

