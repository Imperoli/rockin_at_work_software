/*
 * next_best_view.h
 *
 *      Author: Marco Imperoli
 */

#ifndef NEXT_BEST_VIEW_H_
#define NEXT_BEST_VIEW_H_

#include <Eigen/Dense>
#include "ros/ros.h"
#include "arm_planner/utils.h"
#include "arm_planner/check_IK_feasibility.h"

struct GazeView{
  Eigen::Vector3d target;
  double pitch;
  double distance;
  Eigen::Vector3d ee_position;
  Eigen::Matrix<double,5,1> joints_target;
};

double scoreFunction(GazeView &reference_view, GazeView &view_in)
{
  double max_ee_pos_diff=.2;
  double ee_pos_diff=(view_in.ee_position-reference_view.ee_position).norm();
  double pitch_diff=fabs(view_in.pitch-reference_view.pitch);
  
  double ee_pos_diff_norm=ee_pos_diff/max_ee_pos_diff;

  return (ee_pos_diff_norm+sin(pitch_diff*M_PI/180.f))/2;
}

double evaluateScore(std::vector<GazeView> &previous_gaze_views, GazeView &view)
{
  double score=1000;
  double tmp_score;
  for(int i=0; i<previous_gaze_views.size(); i++){
    if(i==0)
    {
      score=scoreFunction(previous_gaze_views[i], view);
      continue;
    }
    tmp_score=scoreFunction(previous_gaze_views[i], view);
    if(tmp_score<score)
    {
      score=tmp_score;
    }
  }
  return score;
}

bool isNear(GazeView &view_in, GazeView &reference_view)
{
  double ee_pos_diff=(view_in.ee_position-reference_view.ee_position).norm();
  double pitch_diff=fabs(view_in.pitch-reference_view.pitch);
  return (ee_pos_diff<.05&&pitch_diff<10);
}

bool checkIfFeasibleGaze(ros::ServiceClient &ik_client, Eigen::Vector3d &p, double pitch, double roll, double dist, Eigen::Vector3d& ee_pos, Eigen::Matrix<double,5,1> &joints_target)
{
  double yaw=yawFromTarget(p);
  Eigen::Vector3d rpy(roll*M_PI/180.0, (pitch-5)*M_PI/180, yaw);
  Eigen::Vector3d target_cartesian_normal;
  rpyToNorm(rpy,target_cartesian_normal);

  //yaw=2.94961-yaw;
  double d=dist;
  Eigen::Vector3d target;
  Eigen::Vector3d cam_offset(-0.0827602, -0.0941455, -0.0);
  Eigen::Vector3d target2;
  target2(2)=-target2(1)*cos((pitch-5)*M_PI/180.0);
  target2(1)=-target2(0)*sin(yaw);
  target2(0)=-target2(0)*cos(yaw);
  
  target(0)=d*cos(yaw-M_PI)*cos((pitch-5)*M_PI/180.0);
  target(1)=d*sin(yaw-M_PI)*cos((pitch-5)*M_PI/180.0);
  target(2)=d*sin(pitch*M_PI/180.0);
  Eigen::Vector3d target_cartesian_position=p+(target+target2); target_cartesian_position(2)+=.04;
  
  arm_planner::check_IK_feasibility srv;
  srv.request.target_position[0]=target_cartesian_position(0);
  srv.request.target_position[1]=target_cartesian_position(1);
  srv.request.target_position[2]=target_cartesian_position(2);
  
  srv.request.target_normal[0]=target_cartesian_normal(0);
  srv.request.target_normal[1]=target_cartesian_normal(1);
  srv.request.target_normal[2]=target_cartesian_normal(2);
  
  if(!ik_client.call(srv)) return false;
  if(srv.response.feasible)
  {
    ee_pos(0)=srv.response.ee_position[0];
    ee_pos(1)=srv.response.ee_position[1];
    ee_pos(2)=srv.response.ee_position[2];
    for(int i=0; i<5; i++)
      joints_target(i)=srv.response.joints_target[i];
      
    return true;
  }
  return false;
}

bool computePossibleGazeViews(ros::ServiceClient &ik_client,
                              Eigen::Affine3d &init_pose,
                              double d_x, double d_y, double d_z,
                              double max_dist, double min_dist,
                              double max_pitch, double min_pitch,
                              std::vector<GazeView> &possible_gaze_views)
{
  possible_gaze_views.clear();
  int count=0;
  for(double x=-d_x/2; x<=d_x/2; x+=0.02){
    for(double y=-d_y/2; y<=d_y/2; y+=0.03){
      for(double z=-d_z/2; z<=d_z/2; z+=0.03){
        for(double dist=min_dist; dist<=max_dist; dist+=0.05){
          for(double pitch=min_pitch; pitch<=max_pitch; pitch+=5){
            Eigen::Vector3d p(x,y,z);
            p=init_pose*p;
            Eigen::Vector3d ee_pos;
            Eigen::Matrix<double,5,1> joints_target;
            if(checkIfFeasibleGaze(ik_client, p,pitch,90,dist, ee_pos, joints_target))
            {
              GazeView view;
              view.target=p;
              view.pitch=pitch;
              view.distance=dist;
              view.ee_position=ee_pos;
              view.joints_target=joints_target;
              possible_gaze_views.push_back(view);
            }
            count++;
          }
        }
      }
    }
  }
  std::cout<<"total_count: "<<count<<std::endl;
  return true;
}

bool removeNearPossibleGazeViews(std::vector<GazeView> &possible_gaze_views, int idx)
{
  std::vector<GazeView> new_gaze_views;
  GazeView reference_view=possible_gaze_views[idx];
  for(int i=0; i<possible_gaze_views.size(); i++){
    if (i==idx) continue;
    if (isNear(possible_gaze_views[i], reference_view)) continue;
    new_gaze_views.push_back(possible_gaze_views[i]);
  }
  possible_gaze_views=new_gaze_views;
  return true;
}

bool computeNextBestView(std::vector<GazeView> &possible_gaze_views, std::vector<GazeView> &previous_gaze_views, GazeView &best_view)
{
  double max_score=0;
  int max_idx=-1;
  for(int i=0; i<possible_gaze_views.size(); i++){
    double score=evaluateScore(previous_gaze_views, possible_gaze_views[i]);
    if (score>max_score)
    {
      max_score=score;
      max_idx=i;
    }
  }  
  std::cout<<"score: "<<max_score<<std::endl;
  if(max_idx<0) return false;
  best_view=possible_gaze_views[max_idx];
  //removeNearPossibleGazeViews(possible_gaze_views, max_idx);
  return true;
}

void extractNViews(int N, ros::ServiceClient &ik_client,  Eigen::Affine3d &init_pose, std::vector<GazeView> &gaze_views)
{
  double d_x=.02; double d_y=.08; double d_z=.08;
  //double d_x=.12; double d_y=.12; double d_z=.02;
  double max_dist=.24; double min_dist=.20;
  double max_pitch=60; double min_pitch=25;
  
  gaze_views.clear();
  std::vector<GazeView> possible_gaze_views;
  computePossibleGazeViews(ik_client, init_pose, d_x, d_y, d_z, max_dist, min_dist, max_pitch, min_pitch, possible_gaze_views);
  std::cout<<"possible_gaze_views_size: "<<possible_gaze_views.size()<<std::endl;
  for(int i=0; i<N; i++)
  {
    if(i==0)
    {
      if(possible_gaze_views.size()<1) return;
      int idx = rand() % possible_gaze_views.size();
      gaze_views.push_back(possible_gaze_views[idx]);
      //removeNearPossibleGazeViews(possible_gaze_views, idx);
      std::cout<<"new_possible_gaze_views_size: "<<possible_gaze_views.size()<<std::endl;
      std::cout<<"gaze_view_"<<i<<":\n"<<gaze_views[i].target.transpose()<<" "<<gaze_views[i].pitch<<" "<<gaze_views[i].distance<<std::endl;
    }
    else
    {
      GazeView best_view;
      if (!computeNextBestView(possible_gaze_views, gaze_views, best_view)) return ;
      gaze_views.push_back(best_view);
      std::cout<<"new_possible_gaze_views_size: "<<possible_gaze_views.size()<<std::endl;
      std::cout<<"gaze_view_"<<i<<":\n"<<gaze_views[i].target.transpose()<<" "<<gaze_views[i].pitch<<" "<<gaze_views[i].distance<<std::endl;
    }
  }
  return ;
}

void extractNRandomViews(int N, ros::ServiceClient &ik_client,  Eigen::Affine3d &init_pose, std::vector<GazeView> &gaze_views)
{
  double d_x=.02; double d_y=.08; double d_z=.08;
  //double d_x=.12; double d_y=.12; double d_z=.02;
  double max_dist=.30; double min_dist=.15;
  double max_pitch=60; double min_pitch=25;
  
  gaze_views.clear();
  std::vector<GazeView> possible_gaze_views;
  computePossibleGazeViews(ik_client, init_pose, d_x, d_y, d_z, max_dist, min_dist, max_pitch, min_pitch, possible_gaze_views);
  std::cout<<"possible_gaze_views_size: "<<possible_gaze_views.size()<<std::endl;
  for(int i=0; i<N; i++)
  {
    if(possible_gaze_views.size()<1) return;
    int idx = rand() % possible_gaze_views.size();
    gaze_views.push_back(possible_gaze_views[idx]);
    //removeNearPossibleGazeViews(possible_gaze_views, idx);
    std::cout<<"new_possible_gaze_views_size: "<<possible_gaze_views.size()<<std::endl;
    std::cout<<"gaze_view_"<<i<<":\n"<<gaze_views[i].target.transpose()<<" "<<gaze_views[i].pitch<<" "<<gaze_views[i].distance<<std::endl;
  }
  return ;
}

#endif
