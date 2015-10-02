#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"

#include "cv_bridge/cv_bridge.h"
#include <actionlib/server/simple_action_server.h>
#include "youbot_object_recognition/RecognizeObjectAction.h"
#include "mcr_perception_msgs/ObjectList.h"

#include <eigen_conversions/eigen_msg.h>

#include <boost/thread.hpp>
#include <Eigen/Core>

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <limits>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include "ceres/ceres.h"

#include "cv_ext/cv_ext.h"
#include "raster_object_model3D.h"

#include "chamfer_matching.h"
#include "object_recognition.h"

#include <omp.h>

#include "std_srvs/Empty.h"
#include "pointgrey_camera_driver/HDRService.h"

#include "youbot_object_recognition/volume_in_camera_fov.h"

#include "utils.h"
#include "mutual_information_utils.h"

// WARNING DEBUG CODE
cv::Mat _dbg_input_img;
RasterObjectModel3DPtr dbg_obj_model_ptr;


static const double OUTER_POINT_SCORE = 0.6;

ros::ServiceClient client_hdr;
actionlib::SimpleActionServer<youbot_object_recognition::RecognizeObjectAction>* as;
actionlib::SimpleActionServer<youbot_object_recognition::RecognizeObjectAction>* as_detection;
actionlib::SimpleActionServer<youbot_object_recognition::RecognizeObjectAction>* as_nbv;
actionlib::SimpleActionServer<youbot_object_recognition::RecognizeObjectAction>* as_nbv_MI;
std::vector<cv::Mat> images;
cv::Mat_<double> camera_matrix(3,3);
cv::Mat_<double> dist_coeff(5,1);
std::string stl_file, stl_dir;
boost::mutex image_mtx;
bool acquire_image=false;
bool image_acquired=false;

std::vector<double> belief;


void pose2Affine(geometry_msgs::Pose pose, Eigen::Affine3d& T)
{
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  Eigen::Matrix3d R(q);
  Eigen::Vector3d t(pose.position.x, pose.position.y, pose.position.z);
  T.linear()=R; T.translation()=t;
}

double evaluateScore ( cv_ext::ImageStatisticsPtr &img_stats_p,
                       std::vector<cv::Point2f> &raster_pts,
                       const std::vector<float> &normal_directions )
{
  // TODO Use gradient magnitude
  boost::shared_ptr< std::vector<float> > g_dir_p =  img_stats_p->getGradientDirections ( raster_pts );
  boost::shared_ptr< std::vector<float> > g_mag_p =  img_stats_p->getGradientMagnitudes ( raster_pts );
  
  std::vector<float> &g_dir = *g_dir_p;
  std::vector<float> &g_mag = *g_mag_p;

  if ( !g_dir.size() || !g_mag_p->size() )
    return 0;

  double score = 0;
  for ( int i = 0; i < g_dir.size(); i++ )
  {
    float &direction = g_dir[i], magnitude = g_mag[i];
    if ( img_stats_p->outOfImage ( direction ) )
      score += OUTER_POINT_SCORE;
    else
      score += ((magnitude > 0.01)?1.0:magnitude ) * std::abs ( cos ( double ( direction ) - normal_directions[i] ) );
  }

  return score/g_dir.size();
}


void project3dEigenPoints(cv::Mat& K, double scale, std::vector<Eigen::Vector3d>& p_list, std::vector<cv::Point2f>& proj_pts)
{
  proj_pts.clear();
  double fx=K.at<double>(0,0)/scale; double fy=K.at<double>(1,1)/scale; double cx=K.at<double>(0,2)/scale; double cy=K.at<double>(1,2)/scale;
  for(int i=0;i<p_list.size();i++){
    cv::Point2f p;
    p.x=(float)(p_list[i](0)*fx + p_list[i](2)*cx)/p_list[i](2);
    p.y=(float)(p_list[i](1)*fy + p_list[i](2)*cy)/p_list[i](2);
    if(p.x>0&&p.x<1920/scale&&p.y>0&&p.y<1200/scale)
    {
      proj_pts.push_back(p);
    //  std::cout<<"pr_pts: "<<p.x<<" "<<p.y<<std::endl;
    }
  }
  //std::cout<<"number pr_pts: "<<proj_pts.size()<<std::endl;
}

bool check3dEigenPointInImage(const cv::Mat& K, const double scale, const Eigen::Vector3d& p_3d)
{
  if(p_3d(2)<=0) return false;
  double fx=K.at<double>(0,0)/scale; double fy=K.at<double>(1,1)/scale; double cx=K.at<double>(0,2)/scale; double cy=K.at<double>(1,2)/scale;
  Eigen::Vector3d p;
  p(0)=(p_3d(0)*fx + p_3d(2)*cx)/p_3d(2);
  p(1)=(p_3d(1)*fy + p_3d(2)*cy)/p_3d(2);
  if(!(p(0)>100/scale&&p(0)<1800/scale&&p(1)>100/scale&&p(1)<1100/scale))
  {
    return false;
  }
  return true;
}

void computeInitGuess(Eigen::Vector3d& t, Eigen::Matrix3d& R, std::vector<Eigen::Vector3d>& t_list)
{
  t_list.clear();
  t_list.push_back(t);
  
  double l_z=0;double l_y=.05; //.14
  double l_x=.02;
  double step=.02; //.02
  Eigen::Affine3d T; T.linear()=R; T.translation()=t;
  
  for(double x=-.01;x<=l_x;x+=.02){
    for(double y=-(l_y/2);y<=l_y/2;y+=.02){
      for(double z=-(l_z/2);z<=l_z/2;z+=.02){
        
        Eigen::Vector3d p(x,y,z);
        p=T*p;
        t_list.push_back(p);
      }
    }
  }
  
  /////RANDOM/////
  /*for (int i=0;i<5;i++){
    double x=(double)(rand()%200-100)/500.0;
    double y=(double)(rand()%200-100)/500.0;
    double z=(double)(rand()%200-100)/4000.0;
    Eigen::Vector3d p(x,y,z);
    p=T*p;
    t_list.push_back(p);
  }*/
  
  //std::cout<<"number initial guess: "<<t_list.size()<<std::endl;
}

void computeStarOfRotations(Eigen::Matrix3d& initial_orientation, std::vector< Eigen::Quaterniond >& quat_rotations)
{
  Eigen::Vector3d unit_x(initial_orientation.col(2));
  Eigen::Vector3d unit_y(initial_orientation.col(1));
  Eigen::Vector3d unit_z(initial_orientation.col(0));
  quat_rotations.clear();
  std::vector<Eigen::Quaterniond> base_rotations;
  base_rotations.push_back(Eigen::Quaterniond(Eigen::AngleAxisd(0,unit_y)));
  base_rotations.push_back(Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/2,unit_y)));
  base_rotations.push_back(Eigen::Quaterniond(Eigen::AngleAxisd(M_PI,unit_y)));
  base_rotations.push_back(Eigen::Quaterniond(Eigen::AngleAxisd(3*M_PI/2,unit_y)));
  base_rotations.push_back(Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/2,unit_z)));
  base_rotations.push_back(Eigen::Quaterniond(Eigen::AngleAxisd(-M_PI/2,unit_z)));

  for(size_t i=0; i<base_rotations.size();i++)
  {
    Eigen::Matrix3d base_rot=(base_rotations[i].toRotationMatrix()*initial_orientation);
    Eigen::Vector3d rot_ax;
      rot_ax=unit_x;
    
    for(double k=0;k<2*M_PI;k+=2*M_PI/10)
    {
      Eigen::Matrix3d R(Eigen::AngleAxisd(k,rot_ax));
      R=R*base_rot;
      Eigen::Quaterniond q(R);
      quat_rotations.push_back(q);
    }
  }
}

bool acquire_image_cb(std_srvs::Empty::Request  &req,
         std_srvs::Empty::Response &res)
{
  image_mtx.lock();
  
  pointgrey_camera_driver::HDRService srv;

 // srv.request.exposure_times.push_back(0.002);
 // srv.request.exposure_times.push_back(0.005);
 // srv.request.exposure_times.push_back(0.01);
  //srv.request.exposure_times.push_back(0.04);
 // srv.request.exposure_times.push_back(0.08);
  srv.request.exposure_times.push_back(0.1);
  //srv.request.exposure_times.push_back(0.14);
  srv.request.exposure_times.push_back(0.18);
  //srv.request.exposure_times.push_back(0.2);
  srv.request.exposure_times.push_back(0.32);

  
  if (client_hdr.call(srv))
  {
    cv::Mat ldr( srv.response.hdr_img.height, srv.response.hdr_img.width, 
	  cv::DataType<uchar>::type, srv.response.hdr_img.data.data() );
	  
    //hdr_pub.publish(srv.response.hdr_img);
    //cv::imshow("ldr_images", ldr);
    //cv::waitKey(20);
    
    cv::pyrDown ( ldr, ldr );
    images.push_back(ldr.clone());
    image_acquired=true;
    std::cout<<"image hdr acquired!!!!!!!!!!!"<<std::endl;
  }
  else
  {
    ROS_ERROR("Failed to call service hdr_camera_node");
    image_mtx.unlock();
    return false;
  }
  
  image_mtx.unlock();
  return true;
}

int projectNewPoints(const std::vector<cv::Point2f>& proj_pts, float scale, cv::Mat& im)
{
  int new_points_count=0;
  for(int i=0;i<proj_pts.size();i++){
    cv::Point2f p=proj_pts[i];
    p.x/=scale; p.y/=scale;
    if(p.x>0&&p.x<im.cols-0&&p.y>0&&p.y<im.rows-0)
    {
      if(im.at<float>(p)==0)
      {
        im.at<float>(p)=1;
        new_points_count++;
      }
    }
  }
  return new_points_count;
}

int projectNewPoints2(const std::vector<cv::Point2f>& proj_pts, float scale, const cv::Mat& im)
{
  int new_points_count=0;
  cv::Mat im2(im.size(),im.type(), cv::Scalar(0));
  for(int i=0;i<proj_pts.size();i++){
    cv::Point2f p=proj_pts[i];
    p.x/=scale; p.y/=scale;
    if(p.x>100&&p.x<im.cols-100&&p.y>50&&p.y<im.rows-50)
    {
      if(im.at<float>(p)==0&&im2.at<float>(p)==0)
      {
        im2.at<float>(p)=1;
        new_points_count++;
      }
    }
  }
  return new_points_count;
}


void buildReferenceImages(RasterObjectModel3DPtr& obj_model_ptr, const std::vector<ObjectCandidate>& candidates,  const std::vector<Eigen::Affine3d>& previous_views, std::vector<cv::Mat>& reference_images)
{ 
  for(int i=0; i<candidates.size(); i++){
    reference_images.push_back(cv::Mat::zeros(images[0].size(), CV_32FC1));
    for(int v=0; v<previous_views.size(); v++){
      Eigen::Affine3d T=previous_views[v].inverse()*candidates[i].pose;
      Eigen::Quaterniond q(T.linear());
      obj_model_ptr->setModelView(q,T.translation());
      //Eigen::Affine3d view=(previous_views[0].inverse()*previous_views[v]);
      Eigen::Affine3d view=previous_views[0].inverse()*candidates[i].pose;
      Eigen::Quaterniond q_view(view.linear());
      std::vector<cv::Point2f> proj_pts;
      obj_model_ptr->projectRasterPoints (q_view, view.translation(), proj_pts);
     /* cv::cvtColor ( images[0], reference_images[i], cv::COLOR_GRAY2BGR );
      cv_ext::drawPoints ( reference_images[i], proj_pts,cv::Scalar ( 0,0,255 ) );
      cv::imshow ( "dbg", reference_images[i] );
      cv::waitKey(0);*/
      int new_points=projectNewPoints(proj_pts,2,reference_images[i]);
    //  std::cerr<<"total_points: "<<proj_pts.size()<<" ===> new_points: "<<new_points<<std::endl;
    }
  }
}

bool compute_camera_poses_fit_the_volume_cb(youbot_object_recognition::volume_in_camera_fov::Request  &req,
         youbot_object_recognition::volume_in_camera_fov::Response &res)
{
  camera_matrix=( cv::Mat_<double> ( 3,3 ) << 1457.845746, 0, 979.39066, 0, 1371.01823, 633.79483, 0, 0, 1);
  dist_coeff=( cv::Mat_<double> ( 5,1 ) << -0.154272, 0.09142, -0.009540999999999999, -0.002034, 0);
  
  std::vector<Eigen::Affine3d> views;
  std::vector<ObjectCandidate> candidates;
  std::vector<int> filtered_views_indices;
  
  for(int i=0; i<req.candidates.size(); i++){
    Eigen::Affine3d o;
    tf::poseMsgToEigen (req.candidates[i], o);
    ObjectCandidate oc;
    oc.pose=o;
    oc.avgDist=0;
    candidates.push_back(oc);
  }
   
  for(int i=0; i<req.views.size(); i++){
    Eigen::Affine3d v;
    tf::poseMsgToEigen (req.views[i], v);
    views.push_back(v);
  }
  
  if(views.size()<=0||candidates.size()<=0) return false;
  
  for(int v=0; v<views.size(); v++){
    bool whole_volume_in_image=true;
    Eigen::Affine3d view_inv=views[v].inverse();
    for(int c=0; c<candidates.size(); c++){
      Eigen::Vector3d p=candidates[c].pose.translation();
      p=view_inv*p;
      if(!check3dEigenPointInImage(camera_matrix, 2, p)){ whole_volume_in_image=false; break;}
    }
    if(whole_volume_in_image) filtered_views_indices.push_back(v);
  }
  res.filtered_views_indices=filtered_views_indices;
}

void compute_NBV_MI_cb(const youbot_object_recognition::RecognizeObjectGoalConstPtr & goal)
{
  image_mtx.lock();
  
  youbot_object_recognition::RecognizeObjectResult result;
  
  camera_matrix=( cv::Mat_<double> ( 3,3 ) << 1457.845746, 0, 979.39066, 0, 1371.01823, 633.79483, 0, 0, 1);
  dist_coeff=( cv::Mat_<double> ( 5,1 ) << -0.154272, 0.09142, -0.009540999999999999, -0.002034, 0);
  cv_ext::PinholeCameraModel cam_model ( camera_matrix, 1920,1200, 2.0, dist_coeff );
  
  stl_file="/home/spqr/object_recognition/bin/test_images/AX-01b_bearing_box.stl";
  RasterObjectModel3DPtr obj_model_ptr( new RasterObjectModel3D() );
  obj_model_ptr->setCamModel( cam_model );
  obj_model_ptr->setStepMeters ( 0.001 ); //meters
  //obj_model_ptr->setCentroidOrigOffset();
  obj_model_ptr->setBBCenterOrigOffset();
  if ( !obj_model_ptr->setStlFile ( stl_file ) )
  {
    as->setAborted(result);
    image_mtx.unlock();
    cv::destroyAllWindows();
    images.clear();
    return;
  }
  //obj_model_ptr->setMinSegmentsLen(0.01);
  obj_model_ptr->computeRaster();


  std::vector<Eigen::Affine3d> views, previous_views;
  Eigen::Affine3d current_view;
  std::vector<ObjectCandidate> candidates, particles;
  std::vector<cv::Mat> observations;
  //std::vector<double> priors;
  
  cv::Mat precomputed_indices;
  
  for(int i=0; i<goal->initial_guess_list.size(); i++){
    Eigen::Affine3d o;
    tf::poseMsgToEigen (goal->initial_guess_list[i], o);
    ObjectCandidate oc;
    oc.pose=o;
    oc.avgDist=goal->object_guess_avgDist[i];
    candidates.push_back(oc);
  }
  
  for(int i=0; i<goal->views.size(); i++){
    Eigen::Affine3d v;
    tf::poseMsgToEigen (goal->views[i], v);
    views.push_back(v);
  }
  
  for(int i=0; i<goal->previous_views.size(); i++){
    Eigen::Affine3d v;
    tf::poseMsgToEigen (goal->previous_views[i], v);
    previous_views.push_back(v);
  }
  
  tf::poseMsgToEigen (goal->current_view, current_view);
  
  if (belief.size()==0)
    computePriorProb(candidates, belief);
  else
    bayesian_update(obj_model_ptr, cam_model, images[images.size()-1], candidates, current_view, belief);
    
  extractParticles(candidates, belief, particles);
  computeObservations(obj_model_ptr,images[0].size(), particles, views, precomputed_indices, observations);
  
  ///// maximizing mutual information /////
  double max_I=0;
  int max_k=-1;
  for(int k=0; k<views.size(); k++){
    std::vector<std::vector<double> > likelihood;
    std::vector<double> ojsk;
    computeLikelihood(obj_model_ptr, k, particles.size(), observations, precomputed_indices, likelihood);
    computeOjDatoSk(particles.size(), observations.size(), likelihood, ojsk);
    double I=computeIk(particles.size(), observations.size(), likelihood, ojsk);
    if(I>max_I)
    {
      max_I=I;
      max_k=k;
      std::cerr<<"best_MI: "<<max_I<<" ===> best_view_index: "<<max_k<<std::endl;
    } 
  }
  
  image_mtx.unlock();
  result.next_best_view_index=max_k;
  as_nbv_MI->setSucceeded(result);
  std::cerr<<"best_MI: "<<max_I<<" ===> best_view_index: "<<max_k<<std::endl;
}


void compute_NBV_DISOCCLUSION_cb(const youbot_object_recognition::RecognizeObjectGoalConstPtr & goal)
{
  image_mtx.lock();
  
  youbot_object_recognition::RecognizeObjectResult result;
  
  camera_matrix=( cv::Mat_<double> ( 3,3 ) << 1457.845746, 0, 979.39066, 0, 1371.01823, 633.79483, 0, 0, 1);
  dist_coeff=( cv::Mat_<double> ( 5,1 ) << -0.154272, 0.09142, -0.009540999999999999, -0.002034, 0);
  cv_ext::PinholeCameraModel cam_model ( camera_matrix, 1920,1200, 2.0, dist_coeff );
  
  stl_file="/home/spqr/object_recognition/bin/test_images/AX-01b_bearing_box.stl";
  RasterObjectModel3DPtr obj_model_ptr( new RasterObjectModel3D() );
  obj_model_ptr->setCamModel( cam_model );
  obj_model_ptr->setStepMeters ( 0.001 ); //meters
  //obj_model_ptr->setCentroidOrigOffset();
  obj_model_ptr->setBBCenterOrigOffset();
  if ( !obj_model_ptr->setStlFile ( stl_file ) )
  {
    as->setAborted(result);
    image_mtx.unlock();
    cv::destroyAllWindows();
    images.clear();
    return;
  }
  //obj_model_ptr->setMinSegmentsLen(0.01);
  obj_model_ptr->computeRaster();


  std::vector<Eigen::Affine3d> views, previous_views;
  std::vector<ObjectCandidate> candidates;
  std::vector<cv::Mat> reference_images;
  
  for(int i=0; i<goal->initial_guess_list.size(); i++){
    Eigen::Affine3d o;
    tf::poseMsgToEigen (goal->initial_guess_list[i], o);
    ObjectCandidate oc;
    oc.pose=o;
    oc.avgDist=goal->object_guess_avgDist[i];
    candidates.push_back(oc);
  }
  
  
  for(int i=0; i<goal->views.size(); i++){
    Eigen::Affine3d v;
    tf::poseMsgToEigen (goal->views[i], v);
    views.push_back(v);
  }
  
  for(int i=0; i<goal->previous_views.size(); i++){
    Eigen::Affine3d v;
    tf::poseMsgToEigen (goal->previous_views[i], v);
    previous_views.push_back(v);
  }
  
  std::cerr<<"sizes: "<<views.size()<<" "<<previous_views.size()<<" "<<candidates.size()<<std::endl;
  
  buildReferenceImages(obj_model_ptr,candidates, previous_views, reference_images);
  
  Eigen::Affine3d reference_view_inv=previous_views[0].inverse();
  double best_score=0; int best_view_index=0;
  for(int v=0; v<views.size(); v++){
    double view_score=0;
    std::cerr<<"view_index: "<<v<<std::endl;

    for(int i=0; i<candidates.size(); i++){
      Eigen::Affine3d T=views[v].inverse()*candidates[i].pose;
      Eigen::Quaterniond q(T.linear());
      obj_model_ptr->setModelView(q,T.translation());
      //Eigen::Affine3d view=(previous_views[0].inverse()*previous_views[v]);
      Eigen::Affine3d view=reference_view_inv*candidates[i].pose;
      Eigen::Quaterniond q_view(view.linear());
      std::vector<cv::Point2f> proj_pts;
      obj_model_ptr->projectRasterPoints (q_view, view.translation(), proj_pts);

      view_score+=((double)projectNewPoints2(proj_pts,2,reference_images[i])/candidates[i].avgDist);
      //std::cerr<<"total_points: "<<proj_pts.size()<<" ===> score: "<<view_score<<std::endl;
    }
    if (view_score>best_score)
    {
      best_score=view_score;
      best_view_index=v;
      
      std::cerr<<"best_score: "<<best_score<<" ===> best_view_index: "<<best_view_index<<std::endl;
      //cv::imshow ( "dbg", reference_images[0] );
      //cv::waitKey(0);
    }
  }
  image_mtx.unlock();
  result.next_best_view_index=best_view_index;
  as_nbv->setSucceeded(result);

  std::cerr<<"best_score: "<<best_score<<" ===> best_view_index: "<<best_view_index<<std::endl;
}


void localize_object_cb(const youbot_object_recognition::RecognizeObjectGoalConstPtr & goal)
{
  image_mtx.lock();
  
  std::cerr<<"object localization"<<std::endl;
  youbot_object_recognition::RecognizeObjectResult result;

  //stl_file=stl_dir+goal->object_name.data+".stl";
  stl_file="/home/spqr/object_recognition/bin/test_images/AX-01b_bearing_box.stl";

 //////// for simul /////////////////////////////
 /* camera_matrix=( cv::Mat_<double> ( 3,3 ) << 1457.845746, 0, 979.39066, 0, 1371.01823, 633.79483, 0, 0, 1);
  dist_coeff=( cv::Mat_<double> ( 5,1 ) << -0.154272, 0.09142, -0.009540999999999999, -0.002034, 0);
  cv::Mat im_0=cv::imread("/home/spqr/obj_rec_dataset/multi/25/im_0.jpg",cv::IMREAD_GRAYSCALE);
  cv::pyrDown ( im_0, im_0 );
  cv::Mat im_1=cv::imread("/home/spqr/obj_rec_dataset/multi/25/im_1.jpg",cv::IMREAD_GRAYSCALE);
  cv::pyrDown ( im_1, im_1 );
  cv::Mat im_2=cv::imread("/home/spqr/obj_rec_dataset/multi/25/im_2.jpg",cv::IMREAD_GRAYSCALE);
  cv::pyrDown ( im_2, im_2 );
  images.resize(3); images[0]=im_0; images[1]=im_1; images[2]=im_2;*/
  ////////////////
 
  camera_matrix=( cv::Mat_<double> ( 3,3 ) << 1457.845746, 0, 979.39066, 0, 1371.01823, 633.79483, 0, 0, 1);
  dist_coeff=( cv::Mat_<double> ( 5,1 ) << -0.154272, 0.09142, -0.009540999999999999, -0.002034, 0);
  cv_ext::PinholeCameraModel cam_model ( camera_matrix, 1920,1200, 2.0, dist_coeff );
  
  std::vector<Eigen::Affine3d> views;
  std::vector<Eigen::Affine3d> views_in;
  for(int i=0; i<goal->views.size(); i++){
    Eigen::Affine3d v;
    //pose2Affine(goal->views[i], v);
    tf::poseMsgToEigen (goal->views[i], v);
    views_in.push_back(v);
    views.push_back(v);
  }
  std::cerr<<"views.size: "<<views_in.size()<<std::endl;
  
  std::vector<Eigen::Affine3d> candidate_list;
  for(int i=0; i<goal->initial_guess_list.size(); i++){
    Eigen::Affine3d p;
    tf::poseMsgToEigen (goal->initial_guess_list[i], p);
    candidate_list.push_back(p);
  }
  
   std::cerr<<"candidates list size: "<<candidate_list.size()<<std::endl;
  
  std::vector<RasterObjectModel3DPtr> obj_model_ptr_vec;
  obj_model_ptr_vec.resize(images.size());  
  for (int i=0; i<images.size(); i++){
    RasterObjectModel3DPtr obj_model_ptr( new RasterObjectModel3D() );
    obj_model_ptr->setCamModel( cam_model );
    obj_model_ptr->setStepMeters ( 0.001 ); //meters
    //obj_model_ptr->setCentroidOrigOffset();
    obj_model_ptr->setBBCenterOrigOffset();
    if ( !obj_model_ptr->setStlFile ( stl_file ) )
    {
      as->setAborted(result);
      image_mtx.unlock();
      cv::destroyAllWindows();
      images.clear();
      return;
    }
    //obj_model_ptr->setMinSegmentsLen(0.01);
    obj_model_ptr->computeRaster();
    obj_model_ptr_vec[i]=obj_model_ptr;
  }


  std::vector<cv::Mat>  dbg_imgs;
  dbg_imgs.resize(images.size());
  for (int i=0; i<images.size(); i++){
    cv::cvtColor ( images[i], dbg_imgs[i], cv::COLOR_GRAY2BGR );
    
   /* std::stringstream ss; ss<<"im_"<<i<<".jpg";
    cv::imwrite(ss.str(), images[i]);*/
  }
  std::vector<cv::Point2f> proj_pts;
  std::vector<float> normals;
  
  std::vector<cv_ext::ImageStatisticsPtr> img_stats_p_vec;
  img_stats_p_vec.resize(images.size());
  for (int i=0; i<images.size(); i++){
    img_stats_p_vec[i]=cv_ext::ImageStatistics::createImageStatistics ( images[i], true );
  }

  cv_ext::BasicTimer timer;

  std::vector<ImageTensorPtr> dist_map_tensor_ptr_vec;
  dist_map_tensor_ptr_vec.resize(images.size());
  for (int i=0; i<images.size(); i++){
    computeDistanceMapTensor ( images[i], dist_map_tensor_ptr_vec[i] );
    std::cerr<<"computeDistanceMapTensor : "<<timer.elapsedTimeMs() <<std::endl;
  }
  

  MultiViewsDirectionalChamferMatching mvdc_matching(cam_model, dist_map_tensor_ptr_vec, dist_map_tensor_ptr_vec[0]);
  mvdc_matching.setTemplateModelVec( obj_model_ptr_vec );
  mvdc_matching.enableVerbouseMode ( false);
  
  MultiViewsCalibrationDirectionalChamferMatching mvcdc_matching(cam_model, dist_map_tensor_ptr_vec, dist_map_tensor_ptr_vec[0]);
  mvcdc_matching.setTemplateModelVec( obj_model_ptr_vec );
  mvcdc_matching.enableVerbouseMode ( false);
  
  
  Eigen::Affine3d calib; calib.matrix()<<0.999971,   0.0066522,   -0.0038337, -0.000215414,
                                 -0.00653147,     0.999508,    0.0306888,   -0.0100829,
                                  0.00403596,   -0.0306628,     0.999522,   0.00189987,
                                           0,            0,            0,            1;
  Eigen::Affine3d res_offset_T=calib;
  for (int i=0; i<images.size(); i++){
    views[i]=(views_in[i]*res_offset_T).inverse();
  }
  
  double score=0;
  Eigen::Affine3d best_T;
  for(int idx=0; idx<candidate_list.size(); idx++)
  {
    std::cerr<<"register candidate "<<idx<<std::endl;
    Eigen::Quaterniond r_q(candidate_list[idx].linear());
    Eigen::Vector3d t_vec_eigen(candidate_list[idx].translation());
    timer.reset();
    mvdc_matching.performOptimization ( r_q, t_vec_eigen, views );
    std::cerr<<"MultiViewsDirectionalChamferMatching optimize : "<<timer.elapsedTimeMs() <<std::endl;
    
    Eigen::Affine3d res_T;
    res_T.linear()=Eigen::Matrix3d(r_q); res_T.translation()=t_vec_eigen;
    double temp_score=0;
    for (int i=0; i<views.size(); i++){
      Eigen::Affine3d T=views[i]*res_T;
      Eigen::Quaterniond q(T.linear());
      cv::cvtColor ( images[i], dbg_imgs[i], cv::COLOR_GRAY2BGR );
      obj_model_ptr_vec[i]->setModelView(q, T.translation());
      //obj_model_ptr->setModelView(r_q, t_vec_eigen);
      obj_model_ptr_vec[i]->projectRasterPoints ( proj_pts, normals );
      cv_ext::drawPoints ( dbg_imgs[i], proj_pts,cv::Scalar ( 0,0,255 ) );
      std::stringstream ss; ss<<"view_"<<i;
      cv::imshow ( ss.str(), dbg_imgs[i] );
      
      temp_score+=evaluateScore(img_stats_p_vec[i], proj_pts,normals);
    }  
    std::cerr<<"reg_score: "<<temp_score<<std::endl;  
    cv::waitKey(20);
    if(temp_score>score){
      score=temp_score;
      best_T=res_T;
    }
    if (score>2.1) break;
  }
  std::cerr<<"best_score before calibration: "<<score<<std::endl;
  
  
  ///////CALIB/////////////
  
  Eigen::Quaterniond r_q;
  Eigen::Vector3d t_vec_eigen;
  Eigen::Quaterniond r_q_offset;
  Eigen::Vector3d t_offset_eigen;
  
  Eigen::Affine3d res_T=best_T;
  
  /*for(int iter=0; iter<5; iter++)
  {
    for (int i=0; i<images.size(); i++){
     // views[i]=(view_0_old.inverse()*T_ee_arm[i]*T_camera_ee[i]*offset_T).inverse();
      views[i]=(views_in[i]*res_offset_T).inverse();
      //views[i]=(T_ee_arm[i]*T_camera_ee[i]*offset_T);
    }
    std::cerr<<"iter: "<<iter<<std::endl;
    timer.reset();
    r_q=Eigen::Quaterniond(res_T.linear());
    t_vec_eigen=res_T.translation();
    mvdc_matching.performOptimization ( r_q, t_vec_eigen, views );
    std::cerr<<"MultiViewsDirectionalChamferMatching optimize : "<<timer.elapsedTimeMs()<<std::endl;
     
     res_T.linear()=Eigen::Matrix3d(r_q); res_T.translation()=t_vec_eigen;
    for (int i=0; i<views.size(); i++){
      Eigen::Affine3d T=views[i]*res_T;
      Eigen::Quaterniond q(T.linear());
      cv::cvtColor ( images[i], dbg_imgs[i], cv::COLOR_GRAY2BGR );
      obj_model_ptr_vec[i]->setModelView(q, T.translation());
      //obj_model_ptr->setModelView(r_q, t_vec_eigen);
      obj_model_ptr_vec[i]->projectRasterPoints ( proj_pts, normals );
      cv_ext::drawPoints ( dbg_imgs[i], proj_pts,cv::Scalar ( 0,0,255 ) );
      std::stringstream ss; ss<<"view_"<<i;
      cv::imshow ( ss.str(), dbg_imgs[i] );
    }
    
    cv::waitKey(20);


    timer.reset();
    r_q_offset=Eigen::Quaterniond(res_offset_T.linear()); t_offset_eigen=res_offset_T.translation();
    mvcdc_matching.performOptimization ( r_q_offset, t_offset_eigen, views_in, res_T);
    std::cerr<<"MultiViewsCalibrationDirectionalChamferMatching optimize : "<<timer.elapsedTimeMs() <<std::endl;
     
     res_offset_T.linear()=Eigen::Matrix3d(r_q_offset); res_offset_T.translation()=t_offset_eigen;
    for (int i=0; i<views_in.size(); i++){
      Eigen::Affine3d T=(views_in[i]*res_offset_T).inverse()*res_T;
      Eigen::Quaterniond q(T.linear());
      cv::cvtColor ( images[i], dbg_imgs[i], cv::COLOR_GRAY2BGR );
      
      obj_model_ptr_vec[i]->setModelView(q, T.translation());
      //obj_model_ptr->setModelView(r_q, t_vec_eigen);
      obj_model_ptr_vec[i]->projectRasterPoints ( proj_pts, normals );
      cv_ext::drawPoints ( dbg_imgs[i], proj_pts,cv::Scalar ( 0,0,255 ) );
      std::stringstream ss; ss<<"view_"<<i;
      cv::imshow ( ss.str(), dbg_imgs[i] );
    }
    cv::waitKey(20);

  }*/
  
  best_T=res_T;
  for (int i=0; i<images.size(); i++){
    views[i]=(views_in[i]*res_offset_T).inverse();
  }
  ////////////////////////////
  
  //////RIS VIS///////////////
  score=0;
  for (int i=0; i<views.size(); i++){
    Eigen::Affine3d T=views[i]*best_T;
    Eigen::Quaterniond q(T.linear());
    cv::cvtColor ( images[i], dbg_imgs[i], cv::COLOR_GRAY2BGR );
    obj_model_ptr_vec[i]->setModelView(q, T.translation());
    //obj_model_ptr->setModelView(r_q, t_vec_eigen);
    obj_model_ptr_vec[i]->projectRasterPoints ( proj_pts, normals );
    cv_ext::drawPoints ( dbg_imgs[i], proj_pts,cv::Scalar ( 0,255,255 ) );
    std::stringstream ss; ss<<"view_"<<i;
    cv::imshow ( ss.str(), dbg_imgs[i] );
    
    score+=evaluateScore(img_stats_p_vec[i], proj_pts,normals);
  }
  std::cerr<<"Calib:\n"<<res_offset_T.matrix()<<std::endl;
   std::cerr<<"object pose:\n"<<best_T.matrix()<<std::endl;  
  std::cerr<<"best_score after calibration: "<<score<<std::endl;
   
  geometry_msgs::Pose p;
  tf::poseEigenToMsg(best_T,p);
  result.object_pose=p; 
  int key = cv::waitKey();
  switch ( key )
  {
    case 'o': 
      as->setSucceeded(result);
      images.clear();
      image_mtx.unlock();
      return;
      break;
  }
  //////////////////////////////7
  
  as->setAborted(result);
  images.clear();
  image_mtx.unlock();
  
  return;
}


 cv::Mat dbg_img_global;
void detect_object_cb(const youbot_object_recognition::RecognizeObjectGoalConstPtr & goal)
{
  image_mtx.lock();
  
  youbot_object_recognition::RecognizeObjectResult result;
  
  
  cv::Mat_<double> r_vec = ( cv::Mat_<double> ( 3,1 ) << -2.61,0.38,0.15 ),
                   t_vec = ( cv::Mat_<double> ( 3,1 ) << 0.07, -0.01,  0.23 );
                   
    
  std::vector<cv::Mat_<double> > t_vec_list;
  std::vector<cv::Mat_<double> > r_vec_list;
  
  
  //stl_file=stl_dir+goal->object_name.data+".stl";
  stl_file="/home/spqr/object_recognition/bin/test_images/AX-01b_bearing_box.stl";
  
  cv::Mat_<double> prev_r_vec, prev_t_vec;

 //////// for simul /////////////////////////////
  /*camera_matrix=( cv::Mat_<double> ( 3,3 ) << 1457.845746, 0, 979.39066, 0, 1371.01823, 633.79483, 0, 0, 1);
  dist_coeff=( cv::Mat_<double> ( 5,1 ) << -0.154272, 0.09142, -0.009540999999999999, -0.002034, 0);
  cv::Mat im_0=cv::imread("/home/spqr/obj_rec_dataset/multi/25/im_0.jpg",cv::IMREAD_GRAYSCALE);
  cv::pyrDown ( im_0, im_0 );
  images.resize(3); images[0]=im_0;*/
  ////////////////
 
  camera_matrix=( cv::Mat_<double> ( 3,3 ) << 1457.845746, 0, 979.39066, 0, 1371.01823, 633.79483, 0, 0, 1);
  dist_coeff=( cv::Mat_<double> ( 5,1 ) << -0.154272, 0.09142, -0.009540999999999999, -0.002034, 0);
  cv_ext::PinholeCameraModel cam_model ( camera_matrix, 1920,1200, 2.0, dist_coeff );
  
 /* std::vector<Eigen::Affine3d> views;
  std::vector<Eigen::Affine3d> views_in;
  for(int i=0; i<goal->views.size(); i++){
    Eigen::Affine3d v;
    pose2Affine(goal->views[i], v);
    views_in.push_back(v);
    views.push_back(v);
  }*/
  
    RasterObjectModel3DPtr obj_model_ptr( new RasterObjectModel3D() );
    obj_model_ptr->setCamModel( cam_model );
    obj_model_ptr->setStepMeters ( 0.001 ); //meters
    //obj_model_ptr->setCentroidOrigOffset();
    obj_model_ptr->setBBCenterOrigOffset();
    if ( !obj_model_ptr->setStlFile ( stl_file ) )
    {
      as->setAborted(result);
      image_mtx.unlock();
      cv::destroyAllWindows();
      images.clear();
      return;
    }
    //obj_model_ptr->setMinSegmentsLen(0.01);
    obj_model_ptr->computeRaster();


  
  ObjectRecognition obj_rec;
  obj_rec.setCamModel(cam_model);
  
  Eigen::Vector3d r_vec_eigen, t_vec_eigen, r_offset_eigen, t_offset_eigen, t_calib;
  Eigen::Matrix3d initial_rotation, offset_rotation, calib_rot;
  
 
  cv::cvtColor ( images[0], dbg_img_global, cv::COLOR_GRAY2BGR );
    
  std::vector<cv::Point2f> proj_pts;
  std::vector<float> normals;
  
 /* std::vector<cv_ext::ImageStatisticsPtr> img_stats_p_vec;
  img_stats_p_vec.resize(images.size());
  for (int i=0; i<images.size(); i++){
    img_stats_p_vec[i]=cv_ext::ImageStatistics::createImageStatistics ( images[i], true );
  }*/

  
  
  Eigen::Affine3d res_T, res_offset_T, offset_T;
  Eigen::Quaterniond r_q_offset;
  offset_T=Eigen::Affine3d::Identity();
  
  std::vector<std::pair<double,Eigen::Affine3d> > model_views;
  std::vector<Eigen::Quaterniond> quat_rotations;
  
  bool exit_now=false;

  while ( ros::ok() )
  {
    if(exit_now) break;
    
    prev_r_vec = r_vec.clone();
    prev_t_vec = t_vec.clone();
    r_vec_eigen=Eigen::Vector3d(r_vec(0,0),r_vec(1,0),r_vec(2,0));
    if(r_vec_eigen.norm()==0) initial_rotation=Eigen::Matrix3d::Identity();
    else initial_rotation=Eigen::Matrix3d(Eigen::AngleAxisd(r_vec_eigen.norm(),r_vec_eigen/r_vec_eigen.norm()));
    t_vec_eigen=Eigen::Vector3d(t_vec(0,0),t_vec(1,0),t_vec(2,0));
    
    Eigen::Quaterniond r_q(initial_rotation);
    
    Eigen::Affine3d init_T; init_T.linear()=initial_rotation; init_T.translation()=t_vec_eigen;

    //////VISUAL/////////////////
    cv::cvtColor ( images[0],dbg_img_global, cv::COLOR_GRAY2BGR );
    obj_model_ptr->setModelView(r_q, init_T.translation());
    obj_model_ptr->projectRasterPoints ( proj_pts, normals );
    cv_ext::drawPoints ( dbg_img_global, proj_pts,cv::Scalar ( 0,0,255 ) );

    std::vector<Eigen::Vector3d> list_t;
    computeInitGuess(t_vec_eigen, initial_rotation, list_t);
    std::vector<cv::Point2f> pr_pts;
    project3dEigenPoints(camera_matrix,2, list_t, pr_pts);
    cv_ext::drawPoints ( dbg_img_global, pr_pts,cv::Scalar ( 0,255,0 ) );
    
    cv::imshow ( "init_guess_img", dbg_img_global );
    std::cout<<t_vec_eigen.transpose()<<"    "<<r_vec_eigen<<std::endl;
    //////////////////////////////////7
    
   
    int cmd;
    int key = cv::waitKey();
    std::cout<< key <<std::endl;
    switch ( key )
    {
      case 65362:
      case 1113938:
        t_vec.at<double> ( 1,0 ) -= 0.01;
        break;
      case 65364:
      case 1113940:
        t_vec.at<double> ( 1,0 ) += 0.01;
        break;
      case 65361:
      case 1113937:
        t_vec.at<double> ( 0,0 ) -= 0.01;
        break;
      case 65363:
      case 1113939:
        t_vec.at<double> ( 0,0 ) += 0.01;
        break;
      case 65435:
      case 65365:
      case 1114027:
        t_vec.at<double> ( 2,0 ) -= 0.01;
        std::cout<<t_vec.at<double> ( 2,0 ) <<std::endl;
        break;
      case 65434:
      case 65366:
      case 1114029:
        t_vec.at<double> ( 2,0 ) += 0.01;
        std::cout<<t_vec.at<double> ( 2,0 ) <<std::endl;
        break;
      case 'a':
      case 'A':
      case 1048673 :
        r_vec.at<double> ( 1,0 ) += 0.01;
        break;
      case 's':
      case 'S':
      case 1048691 :
        r_vec.at<double> ( 1,0 ) -= 0.01;
        break;
      case 'w':
      case 'W':
      case 1048695 :
        r_vec.at<double> ( 0,0 ) += 0.01;
        break;
      case 'z':
      case 'Z':
      case 1048698 :
        r_vec.at<double> ( 0,0 ) -= 0.01;
        break;
      case 'q':
      case 'Q':
      case 1048689 :
        r_vec.at<double> ( 2,0 ) += 0.01;
        break;
      case 'e':
      case 'E':
      case 1048677 :
        r_vec.at<double> ( 2,0 ) -= 0.01;
        break;
      
      case 'o':
         computeStarOfRotations(initial_rotation,quat_rotations);
         obj_rec.detectObject(obj_model_ptr, images[0], quat_rotations, list_t, model_views);
         for(int i=0; i<model_views.size(); i++){
            Eigen::Quaterniond r_q(model_views[i].second.linear());
            //cv::cvtColor ( images[0], dbg_img_global, cv::COLOR_GRAY2BGR );
            obj_model_ptr->setModelView(r_q, model_views[i].second.translation());
            obj_model_ptr->projectRasterPoints ( proj_pts, normals );
            cv_ext::drawPoints ( dbg_img_global, proj_pts,cv::Scalar ( 255,0,0 ) );
            cv::imshow ( "detection", dbg_img_global );
            cv::waitKey(10);
         }
         for(int i=0; i<model_views.size(); i++){
          geometry_msgs::Pose p;
          tf::poseEigenToMsg(model_views[i].second,p);
          result.object_guess_poses.push_back(p);
          result.object_guess_avgDist.push_back(model_views[i].first);
         }
         image_mtx.unlock();
        // cv::destroyAllWindows();
         as_detection->setSucceeded(result);
         exit_now=true;
        break;
    }
  }
  image_mtx.unlock();
  cv::destroyWindow("init_guess_img");
  cv::destroyAllWindows();
}

int main ( int argc, char** argv )
{
  //stl_dir=argv[1];
  
  ros::init(argc, argv, "arm_camera_objects_recognition_server");
  ros::NodeHandle n;
  
  ros::ServiceServer service = n.advertiseService("acquire_image", acquire_image_cb);
  client_hdr = n.serviceClient<pointgrey_camera_driver::HDRService>("hdr_camera_node");
  
  
  ros::ServiceServer volume_in_cam_fov = n.advertiseService("compute_camera_poses_fit_the_volume", compute_camera_poses_fit_the_volume_cb);
  
  as_nbv=new actionlib::SimpleActionServer<youbot_object_recognition::RecognizeObjectAction>(n, "compute_next_best_view_disocclusion", compute_NBV_DISOCCLUSION_cb, false);
	as_nbv->start();
	
	as_nbv_MI=new actionlib::SimpleActionServer<youbot_object_recognition::RecognizeObjectAction>(n, "compute_next_best_view_mutual_information", compute_NBV_MI_cb, false);
	as_nbv_MI->start();
  
  as_detection=new actionlib::SimpleActionServer<youbot_object_recognition::RecognizeObjectAction>(n, "detect_object", detect_object_cb, false);
	as_detection->start();
   
  as=new actionlib::SimpleActionServer<youbot_object_recognition::RecognizeObjectAction>(n, "localize_object", localize_object_cb, false);
	as->start();
	
	ROS_INFO("camera_arm server ready");

  ros::spin();

  return 0;
}


