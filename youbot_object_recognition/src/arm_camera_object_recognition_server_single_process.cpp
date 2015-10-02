#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
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

#include "object_recognition.h"

#include <omp.h>

#include <boost/filesystem.hpp>



static const double OUTER_POINT_SCORE = 0.6;
actionlib::SimpleActionServer<youbot_object_recognition::RecognizeObjectAction>* as;
cv::Mat image;
cv::Mat_<double> camera_matrix(3,3);
cv::Mat_<double> dist_coeff(5,1);
std::string stl_file, stl_dir;

//RasterObjectModel3DPtr obj_model_ptrs[5];
RasterModelImagePtr model_image_ptrs[3];
std::map<std::string,int> map_models;

void computeInitGuessForSaving(Eigen::Vector3d& t, Eigen::Matrix3d& R, std::vector<Eigen::Vector3d>& t_list)
{
  t_list.clear();
  double l_z=.07;double l_y=.12; //.14
  double l_x=.04;
  double step=.02; //.02
  Eigen::Affine3d T; T.linear()=R; T.translation()=Eigen::Vector3d(0,0,.3);
  for(double x=-.01;x<=l_x;x+=.02){
    for(double y=-(l_y/2);y<=l_y/2;y+=.0175){
      for(double z=-(l_z/2);z<=l_z/2;z+=.02){
        
        Eigen::Vector3d p(x,y,z);
        p=T*p;
        t_list.push_back(p);
      }
    }
  }
  std::cout<<"number initial guess: "<<t_list.size()<<std::endl;
}

void computeStarOfRotations(Eigen::Matrix3d& initial_orientation, std::vector< Eigen::Quaterniond >& quat_rotations)
{
  Eigen::Vector3d unit_x(initial_orientation.col(0));
  Eigen::Vector3d unit_y(initial_orientation.col(1));
  Eigen::Vector3d unit_z(initial_orientation.col(2));
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

void project3dEigenPoints(cv::Mat& K, std::vector<Eigen::Vector3d>& p_list, std::vector<cv::Point2f>& proj_pts)
{
  proj_pts.clear();
  double fx=K.at<double>(0,0); double fy=K.at<double>(1,1); double cx=K.at<double>(0,2); double cy=K.at<double>(1,2);
  for(int i=0;i<p_list.size();i++){
    cv::Point2f p;
    p.x=(float)(p_list[i](0)*fx + p_list[i](2)*cx)/p_list[i](2);
    p.y=(float)(p_list[i](1)*fy + p_list[i](2)*cy)/p_list[i](2);
    if(p.x>0&&p.x<1024&&p.y>0&&p.y<768)
    {
      proj_pts.push_back(p);
    //  std::cout<<"pr_pts: "<<p.x<<" "<<p.y<<std::endl;
    }
  }
  //std::cout<<"number pr_pts: "<<proj_pts.size()<<std::endl;
}

void get_image(sensor_msgs::Image img, 
		   const sensor_msgs::CameraInfo info) {  

    // Get camera info
    int i = 0;
    for(int r = 0; r < 3; r++) {
      for(int c = 0; c < 3; c++, i++) {
        camera_matrix(r, c) = info.K[i];
      }
    }
    camera_matrix(2,2)=1;
    for (int j=0; j<5;j++){
      dist_coeff(j,0)=info.D[j];
    }

    cv_bridge::CvImagePtr im = cv_bridge::toCvCopy(img, img.encoding);
    image=im->image.clone();
}

void recognize_object_cb(const youbot_object_recognition::RecognizeObjectGoalConstPtr & goal)
{
  
  youbot_object_recognition::RecognizeObjectResult result;
  
  
  cv::Mat_<double> t_vec(3,1);
  cv::Mat_<double> r_vec(3,1);  
    
  std::vector<cv::Mat_<double> > t_vec_list;
  std::vector<cv::Mat_<double> > r_vec_list;
  for (int i=0; i<goal->initial_guess_list.size(); i++){
  
    cv::Mat_<double> t_vec0(3,1);
    t_vec0(0,0)=goal->initial_guess_list[i].position.x; t_vec0(1,0)=goal->initial_guess_list[i].position.y; t_vec0(2,0)=goal->initial_guess_list[i].position.z;
    Eigen::Vector3d ax;
    
    //double qx=q(0); double qy=q(1); double qz=q(2); double qw=q(3);
    //double qx=q.x(); double qy=q.y(); double qz=q.z(); double qw=q.w();
    double qx=goal->initial_guess_list[i].orientation.x; double qy=goal->initial_guess_list[i].orientation.y; double qz=goal->initial_guess_list[i].orientation.z; double qw=goal->initial_guess_list[i].orientation.w;
    

    //double qx=q.w(); double qy=q.x(); double qz=q.y(); double qw=q.z();
    ax(0)=qx / sqrt(1-qw*qw);
    ax(1)=qy / sqrt(1-qw*qw);
    ax(2)=qz / sqrt(1-qw*qw);
    ax=ax/ax.norm();
    double angle = 2 * acos(qw);
    ax=ax*angle;
    cv::Mat_<double> r_vec0(3,1);
    r_vec0(0,0)=ax(0); r_vec0(1,0)=ax(1); r_vec0(2,0)=ax(2);
    
   // if(t_vec0(2,0)>goal->initial_guess_list[0].position.z)
   // {
      t_vec_list.push_back(t_vec0);
      r_vec_list.push_back(r_vec0);
   // }
    if(i==0)
    {
      t_vec=t_vec0.clone(); r_vec=r_vec0.clone();
      
    }

  }
  
  //// WARNING
 // std::ofstream of("/home/spqr/object_recognition/init_guess.txt");of << t_vec(0,0)<<" "<<t_vec(1,0)<<" "<<t_vec(2,0)<<"\n"<<r_vec(0,0)<<" "<<r_vec(1,0)<<" "<<r_vec(2,0)<<std::endl;
  
  std::string stl_file_local;
  stl_file_local=stl_dir+goal->object_name.data+".stl";
  
  cv::Mat_<double> prev_r_vec, prev_t_vec;
  
  cv_ext::BasicTimer timer;
  
  get_image(goal->image, goal->camera_info);

  cv_ext::PinholeCameraModel cam_model ( camera_matrix, image.cols,image.rows, 1, dist_coeff );
  
  RasterModelImagePtr model_image_ptr(new RasterModelImage());
  model_image_ptr->storeModelsFromOther(model_image_ptrs[map_models[goal->object_name.data]]);
  

  RasterObjectModel3DPtr obj_model_ptr( new RasterObjectModel3D() );
  obj_model_ptr->setCamModel( cam_model );
  obj_model_ptr->setStepMeters ( 0.001 ); //meters
  //obj_model_ptr->setCentroidOrigOffset();
  obj_model_ptr->setBBCenterOrigOffset();
  if ( !obj_model_ptr->setStlFile ( stl_file_local ) )
  {
    as->setAborted(result);
    return;
  }
  //obj_model_ptr->setMinSegmentsLen(0.01);
  obj_model_ptr->computeRaster();
  
  //obj_model_ptr->loadPrecomputedModelsViews(stl_dir+goal->object_name.data); 
  //obj_model_ptr->storePrecomputedModelsFromOther(obj_model_ptrs[map_models[goal->object_name.data]]);
  
  
  //cvtColor(dbg_input_img,dbg_img, cv::COLOR_GRAY2BGR );
 /* for (int i=0;i<view_idxs.size(); i++)
  {
    Eigen::Quaternion<double> r2; Eigen::Vector3d t2;
    obj_model_ptr->modelView(view_idxs[i], r2,t2 );
    std::vector<cv::Point2f> proj_pts2;
    std::vector<float> normals2;
    obj_model_ptr->projectRasterPoints(view_idxs[i], r2, t2, proj_pts2, normals2);
    cv_ext::drawPoints ( dbg_img, proj_pts2,cv::Scalar (  255, 0, 0 ) );
    cv_ext::showImage(dbg_img, "tmp"  );
  }*/
  

  cv::Mat scaled_img; 
  scaled_img=image.clone();
  
  ObjectRecognition obj_rec;
  obj_rec.setCamModel(cam_model);


  std::vector<Eigen::Vector3d> t_list;
  std::vector< Eigen::Quaterniond > quat_rotations; 
  
  Eigen::Vector3d r_vec_eigen, t_vec_eigen;
  Eigen::Matrix3d initial_rotation;
  
  cv::Mat dbg_img;
  cv::cvtColor ( scaled_img, dbg_img, cv::COLOR_GRAY2BGR );
  std::vector<cv::Point2f> proj_pts;
  std::vector<float> normals;
  
  
  prev_r_vec = r_vec.clone();
  prev_t_vec = t_vec.clone();
  r_vec_eigen=Eigen::Vector3d(r_vec(0,0),r_vec(1,0),r_vec(2,0));
  Eigen::Vector3d t(t_vec(0,0),t_vec(1,0),t_vec(2,0));
  if(r_vec_eigen.norm()==0) initial_rotation=Eigen::Matrix3d::Identity();
  else initial_rotation=Eigen::Matrix3d(Eigen::AngleAxisd(r_vec_eigen.norm(),r_vec_eigen/r_vec_eigen.norm()));
  //initial_rotation=Eigen::AngleAxisd(0,Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(0,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(0,Eigen::Vector3d::UnitZ());
  for(int i=0;i<t_vec_list.size(); i++){
    t_list.push_back(Eigen::Vector3d(t_vec_list[i](0,0), t_vec_list[i](1,0), t_vec_list[i](2,0)));     
  }
        
      //  std::vector<cv::Point2f> pr_pts;
      //  project3dEigenPoints(camera_matrix, t_list, pr_pts);
      //  cv_ext::drawPoints ( dbg_img, pr_pts,cv::Scalar ( 0,255,0 ) );


   // cv::imshow ( "test_dxf", dbg_img );
   // cv::waitKey(10);
   
   /*     Eigen::Quaternion<double> r2(initial_rotation); Eigen::Vector3d t2(t);
    obj_model_ptr->setModelView(r2,t2 );
    std::vector<cv::Point2f> proj_pts2;
    std::vector<float> normals2;
    obj_model_ptr->projectRasterPoints(proj_pts2);
    cv_ext::drawPoints ( dbg_img, proj_pts2,cv::Scalar (  255, 0, 0 ) );
    cv_ext::showImage(dbg_img, "tmp"  );
  
        computeStarOfRotations(initial_rotation,quat_rotations);
        
        /////////// SAVING ///////////////////////77
        std::vector<Eigen::Vector3d> list_t;
        computeInitGuessForSaving(t, initial_rotation, list_t);
        obj_rec.precomputeAndSaveModelViews(obj_model_ptr, quat_rotations, list_t);
        std::vector<cv::Point2f> pr_pts;
        project3dEigenPoints(camera_matrix, list_t, pr_pts);
        cv_ext::drawPoints ( dbg_img, pr_pts,cv::Scalar ( 0,255,0 ) );
        cv::imshow ( "test_dxf", dbg_img );
        cv::waitKey(0);
        return;
       ////////////////////////////////////////////7
    */  
  Eigen::Quaterniond r_out;
  Eigen::Vector3d t_out;
  double score;
  //obj_rec.searchObjectWithInitialGuess(obj_model_ptr, scaled_img,quat_rotations, t_list,  r_out, t_out, score );
  std::string save_dir=stl_dir+"experiment/"+goal->object_name.data+"/";
  boost::filesystem::path dir(save_dir.c_str());
  boost::filesystem::create_directories(dir);
  obj_rec.searchObjectWithInitialGuess(obj_model_ptr, model_image_ptr, scaled_img,quat_rotations, t_list,  r_out, t_out, score, false, save_dir );
  
  if (score>0.6)
  {
    if(goal->object_name.data.compare("EM-01_aid_tray")==0)
    {
      Eigen::Matrix3d R;
      Eigen::Vector3d t_EM(t_out);
      if(r_out.norm()==0) R=Eigen::Matrix3d::Identity();
      else //R=Eigen::Matrix3d(Eigen::AngleAxisd(out_poses[out_poses.size()-1].r.norm(),out_poses[out_poses.size()-1].r/out_poses[out_poses.size()-1].r.norm()));
        R=Eigen::Matrix3d(r_out);
      Eigen::Affine3d T; T.linear()=R; T.translation()=t_EM;
      t_out=T*Eigen::Vector3d(-.0,0.05,0.01);
    }
    int cmd=1;
   // std::cin>>cmd;
    if(cmd==1)
    {
      result.object_list.objects.clear();
      mcr_perception_msgs::Object ob;
      ob.pose.pose.position.x=t_out(0); ob.pose.pose.position.y=t_out(1); ob.pose.pose.position.z=t_out(2);
      tf::quaternionEigenToMsg(r_out, ob.pose.pose.orientation);
      ob.probability=(float) score;
      result.object_list.objects.push_back(ob);
      
      as->setSucceeded(result);
     // image_mtx.unlock();
      cv::destroyAllWindows();
      return;
    }
  }
  as->setAborted(result);
  // image_mtx.unlock();
  cv::destroyAllWindows();
  return;
}

int main ( int argc, char** argv )
{
  stl_dir=argv[1];
  
  ros::init(argc, argv, argv[2]);
  ros::NodeHandle n;

  /*RasterObjectModel3DPtr obj_model_ptr4( new RasterObjectModel3D() );
  obj_model_ptrs[4]=obj_model_ptr4;
  obj_model_ptrs[4]->loadPrecomputedModelsViews(stl_dir+"AX-09_motor_with_gearbox");*/

  /*RasterObjectModel3DPtr obj_model_ptr0( new RasterObjectModel3D() );
  obj_model_ptrs[0]=obj_model_ptr0;
  obj_model_ptrs[0]->loadPrecomputedModelsViews(stl_dir+"AX-01b_bearing_box");
  
  RasterObjectModel3DPtr obj_model_ptr1( new RasterObjectModel3D() );
  obj_model_ptrs[1]=obj_model_ptr1;
  obj_model_ptrs[1]->loadPrecomputedModelsViews(stl_dir+"AX-01_bearing_box");
  
  RasterObjectModel3DPtr obj_model_ptr2( new RasterObjectModel3D() );
  obj_model_ptrs[2]=obj_model_ptr2;
  obj_model_ptrs[2]->loadPrecomputedModelsViews(stl_dir+"AX-03_axis");
  
  RasterObjectModel3DPtr obj_model_ptr3( new RasterObjectModel3D() );
  obj_model_ptrs[3]=obj_model_ptr3;
  obj_model_ptrs[3]->loadPrecomputedModelsViews(stl_dir+"EM-01_aid_tray");*/
  
  RasterModelImagePtr model_image_ptr0(new RasterModelImage());
  model_image_ptrs[0]=model_image_ptr0;
  model_image_ptrs[0]->loadModels(stl_dir+"AX-01b_bearing_box_normal");
  
  RasterModelImagePtr model_image_ptr1(new RasterModelImage());
  model_image_ptrs[1]=model_image_ptr1;
  model_image_ptrs[1]->loadModels(stl_dir+"AX-01_bearing_box_normal");
  
  /*RasterModelImagePtr model_image_ptr2(new RasterModelImage());
  model_image_ptrs[2]=model_image_ptr2;
  model_image_ptrs[2]->loadModels(stl_dir+"AX-03_axis_normal");*/
  
  RasterModelImagePtr model_image_ptr2(new RasterModelImage());
  model_image_ptrs[2]=model_image_ptr2;
  model_image_ptrs[2]->loadModels(stl_dir+"EM-01_aid_tray_normal");
  
 // map_models.insert ( std::pair<std::string,int>("AX-09_motor_with_gearbox",4) );
  map_models.insert ( std::pair<std::string,int>("AX-01b_bearing_box",0) );
  map_models.insert ( std::pair<std::string,int>("AX-01_bearing_box",1) );
  //map_models.insert ( std::pair<std::string,int>("AX-03_axis",2) );
  map_models.insert ( std::pair<std::string,int>("EM-01_aid_tray",2) );

  as=new actionlib::SimpleActionServer<youbot_object_recognition::RecognizeObjectAction>(n, argv[2], recognize_object_cb, false);
	as->start();
	
	ROS_INFO("camera_arm single processes server ready");

  ros::spin();

  return 0;
}


