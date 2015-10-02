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
#include "arm_planner/get_ee_pose.h"

std::string ee_filename="/home/spqr/catkin_ws/src/youbot_manipulation/possible_ee_poses.txt";

struct GazeView{
  Eigen::Vector3d target;
  double pitch;
  double distance;
  Eigen::Vector3d ee_position;
  Eigen::Matrix<double,5,1> joints_target;
};

struct View{
  Eigen::Affine3d camera_pose;
  Eigen::Matrix<double,5,1> joints_target;
};

struct ObjectCandidate{
  Eigen::Affine3d pose;
  double avgDist;
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

void readPossibleViews(std::vector<View> &possible_views)
{
  possible_views.clear();
  
  Eigen::Affine3d camera_calib;
  Eigen::Matrix4d calib; calib<<-0.0456833,  0.0832917,   0.993933, -0.0948032,
                                  -0.00206125,     1.0054, -0.0769653,  -0.080637,
                                   -0.987446,  0.0358595, -0.0356085, 0.00759732,
                                           0 ,         0,          0,          1;
  camera_calib.matrix()=calib;
  
  std::ifstream file_ee(ee_filename.c_str());
  double d;
  while(file_ee >> d)
  {
    View v;
    for(int i=0; i<5; i++){
      v.joints_target(i)=d;
      file_ee >> d;
    }
    Eigen::Affine3d ee_pose;
    
    for(int i=0; i<3; i++){
        ee_pose.translation()(i)=d;
        file_ee >> d;
    }
    
    for(int r=0; r<3; r++){
      for(int c=0; c<3; c++){
        ee_pose.linear()(r,c)=d;
        if(r==2&&c==2) continue;
        file_ee >> d;
      }
    }
    
    v.camera_pose=ee_pose*camera_calib;
    possible_views.push_back(v);
  }
  std::cout<<"poss_views: "<<possible_views.size()<<std::endl;
}

bool checkIfFeasible(ros::ServiceClient &ik_client, ros::ServiceClient &ee_pose_client, Eigen::Vector3d &p, double pitch, double roll, Eigen::Matrix<double,5,1> &joints_target)
{
  double yaw=yawFromTarget(p);
  Eigen::Vector3d rpy(roll*M_PI/180.0, pitch*M_PI/180, yaw);
  Eigen::Vector3d target_cartesian_normal;
  rpyToNorm(rpy,target_cartesian_normal);

  Eigen::Vector3d target_cartesian_position=p;
  
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
    
    for(int i=0; i<5; i++)
      joints_target(i)=srv.response.joints_target[i];
      
    return true;
  }
  return false;
}

void computeFOVViews(const Eigen::Affine3d& init_guess, std::vector<View> & good_views)
{
  good_views.clear();
  std::vector<View> possible_views;
  readPossibleViews(possible_views);
  
  Eigen::Matrix3d K; K<<1457.845746, 0, 979.39066, 0, 1371.01823, 633.79483, 0, 0, 1;
  
  Eigen::Vector3d p=init_guess.translation();
  for(int i=0;i<possible_views.size();i++)
  {
    Eigen::Vector3d _p=possible_views[i].camera_pose.inverse()*p;
    if(_p(2)<=0) continue;
    Eigen::Vector3d p_proj=K*_p;
    p_proj=p_proj/p_proj(2);
    if(p_proj(0)<400||p_proj(0)>1400 || p_proj(1)<200||p_proj(1)>900) continue;
    
    good_views.push_back(possible_views[i]);
  }  

  std::cout<<"fov_views: "<<good_views.size()<<std::endl;
}

bool isGoodView(View v, std::vector<View> &prev_views)
{
  double thresh=.2;
  for (int i=0; i<prev_views.size(); i++)
  {
    Eigen::Vector3d dir1=v.camera_pose.linear().col(2);  
    Eigen::Vector3d dir2=prev_views[i].camera_pose.linear().col(2);  
    double dist=(v.camera_pose.translation()-prev_views[i].camera_pose.translation()).norm();
    double cos_ang=fabs(dir1.dot(dir2));
    //std::cout<<(dist+(1-cos_ang*cos_ang))<<std::endl;
    if((dist+(1-cos_ang*cos_ang))<thresh) return false;
  }
  return true;
}

void computeGoodViews(const std::vector<View> &fov_views, std::vector<View> &prev_views,  std::vector<View> & good_views)
{
  good_views.clear();
  
  for(int i=0;i<fov_views.size();i++)
  {

    if(isGoodView(fov_views[i],prev_views))
      good_views.push_back(fov_views[i]);
  }  

  std::cout<<"good_views: "<<good_views.size()<<std::endl;
}

void computeNGoodViews(int N, const std::vector<View> &fov_views, std::vector<View> &prev_views,  std::vector<View> & n_good_views)
{
  std::vector<View> good_views;
  for(int i=0;i<fov_views.size();i++)
  {

    if(isGoodView(fov_views[i],prev_views))
      good_views.push_back(fov_views[i]);
  }
  
  n_good_views.clear();
  for(int i=0;i<N&&i<good_views.size();i++){
    int rand_idx=rand() % good_views.size();
    n_good_views.push_back(good_views[rand_idx]);
    good_views.erase (good_views.begin()+rand_idx);
  }  

  std::cerr<<"good_views: "<<good_views.size()<<std::endl;
}

void computeGeometricNBV(const std::vector<View> &volume_views,  std::vector<View> & views_out)
{
  views_out.clear();
  int first_idx=-1; int second_idx=-1;
  double sq_dist=0;
  for(int i=0;i<volume_views.size();i++){
    Eigen::Vector3d v1=volume_views[i].camera_pose.translation();
    for(int j=0;j<volume_views.size(); j++){
      if(i==j) continue;
      Eigen::Vector3d v2=volume_views[j].camera_pose.translation();
      double sq_dist_temp=(v2-v1).squaredNorm();
      if(sq_dist_temp>sq_dist)
      {
        sq_dist=sq_dist_temp;
        first_idx=i;
        second_idx=j;
      }
    }
  }
  if (sq_dist==0) return;
  views_out.push_back(volume_views[first_idx]); views_out.push_back(volume_views[second_idx]);
}

/*void extractGoodViewsDegug(std::vector<View> &good_views)
{
  std::vector<View> prev_views;

  std::vector<View> possible_views;
  readPossibleViews(possible_views);
  
  std::vector<View> fov_views;
  computeFOVViews(possible_views,  fov_views);
  
  int rand_idx=rand() % fov_views.size();
  View first=fov_views[rand_idx];
  
  prev_views.push_back(first);
  
  computeGoodViews(fov_views, prev_views, good_views);
  
  rand_idx=rand() % good_views.size();
  View second=good_views[rand_idx];
  
  prev_views.push_back(second);
  
  computeGoodViews(fov_views, prev_views, good_views);
  
  rand_idx=rand() % good_views.size();
  View third=good_views[rand_idx];
  
  prev_views.push_back(third);
  
  good_views=prev_views;
}*/

/*void extractNBestViews(int N, std::vector<View> &best_views)
{
  std::vector<View> prev_views,good_views, few_good_views;

  std::vector<View> possible_views;
  readPossibleViews(possible_views);
  std::vector<View> fov_views;
  computeFOVViews(possible_views,  fov_views);
  int rand_idx=rand() % fov_views.size();
  prev_views.push_back(fov_views[rand_idx]);
  
  for(int iter=0; iter<N; iter++)
  {
    few_good_views.clear();
    computeGoodViews(fov_views, prev_views, good_views);
    for(int i=0;i<100&&i<good_views.size();i++){
      rand_idx=rand() % good_views.size();
      few_good_views.push_back(good_views[rand_idx]);
      good_views.erase (good_views.begin()+rand_idx);
    }
    
    std::cout<<"few_good_views: "<<few_good_views.size()<<std::endl;
    double score=0; double tmp_score; int idx=-1;
    for(int i=0;i<few_good_views.size();i++){
      tmp_score=evaluateViewScore(few_good_views[i], detection_candidates);
      if(tmp_score>score){
        score=tmp_score;
        idx=i;
      }
    }
    
  }
  
  good_views=prev_views;
}*/

void extractNRandomViews(int N, const std::vector<View> &fov_views, std::vector<View> &views)
{
  views.clear();
  for(int iter=0; iter<N; iter++)
  {
    int rand_idx=rand() % fov_views.size();
    views.push_back(fov_views[rand_idx]);
  }
}


bool computePossibleViews(ros::ServiceClient &ik_client,
                              ros::ServiceClient &ee_pose_client,
                              Eigen::Affine3d &init_pose,
                              std::vector<GazeView> &possible_gaze_views)
{
  std::ofstream file_ee(ee_filename.c_str());
  
  possible_gaze_views.clear();
  int count=0; int count_possible=0; int count_nan=0;
  for(double x=-.35; x<=.15; x+=0.04){
    for(double y=-.45; y<=-.2; y+=0.04){
      for(double z=0.05; z<.4; z+=0.04){
        std::cout<<x<<" "<<y<<" "<<z<<" "<<count_possible<<std::endl;
        for(double pitch=2;pitch<=90;pitch+=5){
          for(double roll=2;roll<360;roll+=22.5){
            Eigen::Vector3d p(x,y,z);
            Eigen::Matrix<double,5,1> joints_target;
            if(checkIfFeasible(ik_client, ee_pose_client, p,pitch,roll, joints_target))
            { 
              arm_planner::get_ee_pose s;
              for(int i=0; i<5; i++)
                s.request.joint_angles[i]=joints_target(i);
                
              if(!ee_pose_client.call(s)) continue;
              if(!isnan(s.response.ee_pose[0]))
              {
                Eigen::Affine3d ee_pose;
                int i=0;
                for(int r=0; r<4; r++){
                  for(int c=0; c<4; c++){
                    ee_pose.matrix()(r,c)=s.response.ee_pose[i];
                    i++;
                  }
                }
                file_ee<<joints_target.transpose()<<std::endl;
                file_ee<<ee_pose.translation().transpose()<<std::endl;
                file_ee<<ee_pose.linear()<<std::endl;
                count_possible++;
              }
                            
            /*  GazeView view;
              view.target=p;
              view.pitch=pitch;
              //view.distance=0;
              //view.ee_position=ee_pos;
              view.joints_target=joints_target;
              possible_gaze_views.push_back(view);*/
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

void extractNViews(int N, ros::ServiceClient &ik_client, ros::ServiceClient &ee_pose_client, Eigen::Affine3d &init_pose, std::vector<GazeView> &gaze_views)
{
  
  gaze_views.clear();
  std::vector<GazeView> possible_gaze_views;
  computePossibleViews(ik_client,ee_pose_client, init_pose, possible_gaze_views);
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

/*void extractNRandomViews(int N, ros::ServiceClient &ik_client,  Eigen::Affine3d &init_pose, std::vector<GazeView> &gaze_views)
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
}*/

#endif
