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
#include "ceres/ceres.h"

#include "cv_ext/cv_ext.h"
#include "raster_object_model3D.h"

#include "chamfer_matching.h"
#include "object_recognition.h"

#include <omp.h>

#include "std_srvs/Empty.h"
#include "pointgrey_camera_driver/HDRService.h"

// WARNING DEBUG CODE
cv::Mat _dbg_input_img;
RasterObjectModel3DPtr dbg_obj_model_ptr;

/*struct PoseCandidate
{
  double score;
  Eigen::Quaterniond r;
  Eigen::Vector3d t;
};*/

static const double OUTER_POINT_SCORE = 0.6;

ros::ServiceClient client_hdr;
actionlib::SimpleActionServer<youbot_object_recognition::RecognizeObjectAction>* as;
std::vector<cv::Mat> images;
cv::Mat_<double> camera_matrix(3,3);
cv::Mat_<double> dist_coeff(5,1);
std::string stl_file, stl_dir;
boost::mutex image_mtx;
bool acquire_image=false;
bool image_acquired=false;


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


static void help()
{

  std::cout << "a-s-q-e-z-w handle the rotation through axis-angle notation " << std::endl;
  std::cout << "F1 chamfer single matching optimization in the position indicate by the user" << std::endl;
  std::cout << "F2 directional single chamfer matching optimization in the position indicate by the user" << std::endl;
  std::cout << "F5 chamfer local matching optimization in the position indicate by the user" << std::endl;
  std::cout << "F6 directional local chamfer matching optimization in the position indicate by the user" << std::endl;
  std::cout << "F11 chamfer global matching optimization in the position indicate by the user" << std::endl;
  std::cout << "F12 directional global chamfer matching optimization in the position indicate by the user" << std::endl;
  std::cout << "g evaluate score of the model in the position indicate by the user" << std::endl;
  std::cout << "i show the icosphere and points of it considered" << std::endl;
  std::cout << "t matching of the all possibile rotations of model in the whole image" << std::endl;
  std::cout << "y matching of model in the all possibile rotations in the position indicate by the user" << std::endl;
  std::cout << "p save image as showed" << std::endl;
  std::cout << "r set the model to the initial position" << std::endl;
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

void readConfig(std::string filename,cv::Mat& cam_matrix, cv::Mat_<double>& t_vec, cv::Mat_<double>& r_vec)
{
  std::ifstream in_f(filename.c_str());
  for(int i=0;i<3;i++){
    for (int j=0; j<3;j++){
      double v=0;
      in_f >> v; cam_matrix.at<double>(i,j)=v;
    }
  }
  in_f>>t_vec(0,0); in_f>>t_vec(1,0); in_f>>t_vec(2,0); 
  Eigen::Vector4d q_in,q;
 // Eigen::Quaterniond q;
  in_f>>q_in(0); in_f>>q_in(1); in_f>>q_in(2); in_f>>q_in(3);
  //Eigen::Quaterniond q_in2(q_in(3), q_in(0), q_in(1), q_in(2));
  //q=Eigen::Quaterniond(Eigen::AngleAxisd(q_in2)*Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY()));
  Eigen::Vector3d ax;
  q=q_in;
  double qx=q(0); double qy=q(1); double qz=q(2); double qw=q(3);
  //double qx=q.x(); double qy=q.y(); double qz=q.z(); double qw=q.w();
  ax(0)=qx / sqrt(1-qw*qw);
  ax(1)=qy / sqrt(1-qw*qw);
  ax(2)=qz / sqrt(1-qw*qw);
  double angle = 2 * acos(qw);
  ax=ax*angle;
  r_vec(0,0)=ax(0); r_vec(1,0)=ax(1); r_vec(2,0)=ax(2);
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

/*void image_callback(const sensor_msgs::Image::ConstPtr& img, 
		   const sensor_msgs::CameraInfo::ConstPtr& info) {  

  
  image_mtx.lock();
  if(acquire_image)
  {
    // Get camera info
    int i = 0;
    for(int r = 0; r < 3; r++) {
      for(int c = 0; c < 3; c++, i++) {
        camera_matrix(r, c) = info->K[i];
      }
    }
    camera_matrix(2,2)=1;
    for (int j=0; j<5;j++){
      dist_coeff(j,0)=info->D[j];
    }

    cv_bridge::CvImagePtr im = cv_bridge::toCvCopy(img, img->encoding);
    images.push_back(im->image.clone());
  }
  
  acquire_image=false;
  image_acquired=true;
  image_mtx.unlock();
}*/


ImageTensorPtr dist_map_tensor_ptr;
cv::Mat scaled_img;
cv_ext::ImageStatisticsPtr img_stats_p;

void recognize_object_cb(const youbot_object_recognition::RecognizeObjectGoalConstPtr & goal)
{
  image_mtx.lock();
  
  youbot_object_recognition::RecognizeObjectResult result;
  
  
  cv::Mat_<double> r_vec = ( cv::Mat_<double> ( 3,1 ) << 0,0,0 ),
                   t_vec = ( cv::Mat_<double> ( 3,1 ) << -.1,-.45,.0 );
                   
    
  std::vector<cv::Mat_<double> > t_vec_list;
  std::vector<cv::Mat_<double> > r_vec_list;
 /* for (int i=0; i<goal->initial_guess_list.size(); i++){
  
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

  }*/
  
  
  stl_file=stl_dir+goal->object_name.data+".stl";
  
  cv::Mat_<double> prev_r_vec, prev_t_vec;

  camera_matrix=( cv::Mat_<double> ( 3,3 ) << 1457.845746, 0, 979.39066, 0, 1371.01823, 633.79483, 0, 0, 1);
  dist_coeff=( cv::Mat_<double> ( 5,1 ) << -0.154272, 0.09142, -0.009540999999999999, -0.002034, 0);
 
  cv_ext::PinholeCameraModel cam_model ( camera_matrix, 1920,1200, 2.0, dist_coeff );
  
  std::vector<Eigen::Affine3d> views;
  std::vector<Eigen::Affine3d> views_in;
  for(int i=0; i<goal->views.size(); i++){
    Eigen::Affine3d v;
    pose2Affine(goal->views[i], v);
    views_in.push_back(v);
    views.push_back(v);
  }
  
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
  
 // ObjectRecognition obj_rec;
  
 // obj_rec.setCamModel(cam_model);
  
  Eigen::Vector3d r_vec_eigen, t_vec_eigen, r_offset_eigen, t_offset_eigen;
  Eigen::Matrix3d initial_rotation, offset_rotation;
  
  
  cv::Mat dbg_img;
  std::vector<cv::Mat>  dbg_imgs;
  dbg_imgs.resize(images.size());
  for (int i=0; i<images.size(); i++){
    std::stringstream ss; ss<<"/home/spqr/obj_rec_dataset/multi/im_"<<i<<".jpg";
    cv::imwrite(ss.str(), images[i]);
  
    cv::cvtColor ( images[i], dbg_imgs[i], cv::COLOR_GRAY2BGR );
    cv::pyrDown ( images[i], images[i]);

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
    std::cout<<"computeDistanceMapTensor : "<<timer.elapsedTimeMs() <<std::endl;
  }
  

  MultiViewsDirectionalChamferMatching mvdc_matching(cam_model, dist_map_tensor_ptr_vec, dist_map_tensor_ptr_vec[0]);
  mvdc_matching.setTemplateModelVec( obj_model_ptr_vec );
  mvdc_matching.enableVerbouseMode ( false);
  
  MultiViewsCalibrationDirectionalChamferMatching mvcdc_matching(cam_model, dist_map_tensor_ptr_vec, dist_map_tensor_ptr_vec[0]);
  mvcdc_matching.setTemplateModelVec( obj_model_ptr_vec );
  mvcdc_matching.enableVerbouseMode ( false);
  
  DirectionalChamferMatching dc_matching(cam_model, dist_map_tensor_ptr_vec[0] );
  dc_matching.setTemplateModel( obj_model_ptr_vec[0] );
  dc_matching.enableVerbouseMode ( false);
  
  Eigen::Affine3d res_T, res_offset_T, offset_T;
  Eigen::Quaterniond r_q_offset;
  Eigen::Matrix4d calib; calib<< 0.999063,  -0.0431889, -0.00296243,  0.00411967,
                                 0.043204,    0.999052,  0.00524909,  -0.0029362,
                               0.00273292, -0.00537216,    0.999982, -0.00867065,
                                        0,           0,           0,           1;
  //offset_T=Eigen::Affine3d::Identity();
  offset_T.matrix()=calib;

  while ( ros::ok() )
  {
    for (int i=0; i<images.size(); i++){
     // views[i]=(view_0_old.inverse()*T_ee_arm[i]*T_camera_ee[i]*offset_T).inverse();
      views[i]=(views_in[i]*offset_T).inverse();
      //views[i]=(T_ee_arm[i]*T_camera_ee[i]*offset_T);
    }
  
    prev_r_vec = r_vec.clone();
    prev_t_vec = t_vec.clone();
    r_vec_eigen=Eigen::Vector3d(r_vec(0,0),r_vec(1,0),r_vec(2,0));
    if(r_vec_eigen.norm()==0) initial_rotation=Eigen::Matrix3d::Identity();
    else initial_rotation=Eigen::Matrix3d(Eigen::AngleAxisd(r_vec_eigen.norm(),r_vec_eigen/r_vec_eigen.norm()));
    t_vec_eigen=Eigen::Vector3d(t_vec(0,0),t_vec(1,0),t_vec(2,0));
    
    Eigen::Quaterniond r_q(initial_rotation);
    
    Eigen::Affine3d init_T; init_T.linear()=initial_rotation; init_T.translation()=t_vec_eigen;

    for (int i=0; i<views.size(); i++){
      Eigen::Affine3d T=views[i]*init_T;
      Eigen::Quaterniond q(T.linear());
      cv::cvtColor ( images[i], dbg_imgs[i], cv::COLOR_GRAY2BGR );
      obj_model_ptr_vec[i]->setModelView(q, T.translation());
      obj_model_ptr_vec[i]->projectRasterPoints ( proj_pts, normals );
      cv_ext::drawPoints ( dbg_imgs[i], proj_pts,cv::Scalar ( 0,0,255 ) );
      std::stringstream ss; ss<<"view_"<<i;
      cv::imshow ( ss.str(), dbg_imgs[i] );
    }
   
    std::vector<Eigen::Affine3d> camera_poses;
    std::vector<Eigen::Affine3d> v;
    
   
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
      case 'k':
        res_offset_T=offset_T;
        for(int iter=0; iter<6; iter++)
        {
          for (int i=0; i<images.size(); i++){
           // views[i]=(view_0_old.inverse()*T_ee_arm[i]*T_camera_ee[i]*offset_T).inverse();
            views[i]=(views_in[i]*res_offset_T).inverse();
            //views[i]=(T_ee_arm[i]*T_camera_ee[i]*offset_T);
          }
          timer.reset();

          mvdc_matching.performOptimization ( r_q, t_vec_eigen, views );
          std::cout<<"MultiViewsDirectionalChamferMatching optimize : "<<timer.elapsedTimeMs() <<std::endl;
           
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
          
          cv::waitKey(0);
          
          camera_poses.clear();
          for (int i=0; i<images.size(); i++){
            Eigen::Affine3d T=views_in[i];
            camera_poses.push_back(T);
          }

          /*std::vector<Eigen::Affine3d> in_calibration;
          std::vector<int> indxs;
          int size_calib=2;
          in_calibration.resize(size_calib);
          for(int i=0;i<size_calib;i++){
            int idx = rand() % camera_poses.size();
            bool contains=false;
            for(int j=0; j<indxs.size(); j++){
              if (indxs[j]==idx) contains=true;
            }
            if (contains){
              i--;
              continue;
            }
            indxs.push_back(idx);
            in_calibration[i]=camera_poses[i];
          }*/

          timer.reset();
          r_q_offset=Eigen::Quaterniond(res_offset_T.linear()); t_offset_eigen=res_offset_T.translation();
          mvcdc_matching.performOptimization ( r_q_offset, t_offset_eigen, camera_poses, res_T);
          //mvcdc_matching.performOptimization ( r_q_offset, t_offset_eigen, in_calibration, res_T);
          std::cout<<"MultiViewsCalibrationDirectionalChamferMatching optimize : "<<timer.elapsedTimeMs() <<std::endl;
           
           res_offset_T.linear()=Eigen::Matrix3d(r_q_offset); res_offset_T.translation()=t_offset_eigen;
          for (int i=0; i<camera_poses.size(); i++){
            Eigen::Affine3d T=(camera_poses[i]*res_offset_T).inverse()*res_T;
            Eigen::Quaterniond q(T.linear());
            cv::cvtColor ( images[i], dbg_imgs[i], cv::COLOR_GRAY2BGR );
            obj_model_ptr_vec[i]->setModelView(q, T.translation());
            //obj_model_ptr->setModelView(r_q, t_vec_eigen);
            obj_model_ptr_vec[i]->projectRasterPoints ( proj_pts, normals );
            cv_ext::drawPoints ( dbg_imgs[i], proj_pts,cv::Scalar ( 0,0,255 ) );
            std::stringstream ss; ss<<"view_"<<i;
            cv::imshow ( ss.str(), dbg_imgs[i] );
          }
          cv::waitKey(0);

        }
        
        break;
      case 'o':
        
        timer.reset();
        //dc_matching.performOptimization ( r_vec, t_vec );
        v.clear(); v.push_back(views[0]);
        mvdc_matching.performOptimization ( r_q, t_vec_eigen, v );
        std::cout<<"DirectionalChamferMatching optimize : "<<timer.elapsedTimeMs() <<std::endl;

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
        
        cv::waitKey(0);
        
        break;
        
        
      case 'm':
        
          for (int i=0; i<images.size(); i++){
           // views[i]=(view_0_old.inverse()*T_ee_arm[i]*T_camera_ee[i]*offset_T).inverse();
            views[i]=(views_in[i]*offset_T).inverse();
            //views[i]=(T_ee_arm[i]*T_camera_ee[i]*offset_T);
          }
          timer.reset();
          mvdc_matching.performOptimization ( r_q, t_vec_eigen, views );
          std::cout<<"MultiViewsDirectionalChamferMatching optimize : "<<timer.elapsedTimeMs() <<std::endl;
           
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
          
          cv::waitKey(0);
        
        break;
        
      case ',':
        offset_T=res_offset_T;
        std::cout<<"calib:\n"<<offset_T.matrix()<<std::endl;
        break;
        
      case '0':
        as->setAborted(result);
        image_mtx.unlock();
        cv::destroyAllWindows();
        images.clear();
        return;
        break;
        
      case '1':
        result.object_list.objects.clear();
        mcr_perception_msgs::Object ob;
        Eigen::Vector3d t_out=res_T.translation();
        Eigen::Quaterniond r_out(res_T.linear());
        ob.pose.pose.position.x=t_out(0); ob.pose.pose.position.y=t_out(1); ob.pose.pose.position.z=t_out(2);
        tf::quaternionEigenToMsg(r_out, ob.pose.pose.orientation);
        ob.probability=(float) 1;
        result.object_list.objects.push_back(ob);
        
        as->setSucceeded(result);
       // image_mtx.unlock();
        cv::destroyAllWindows();
        return;
        break;
    }
  }
}

int main ( int argc, char** argv )
{
  stl_dir=argv[1];
  
  ros::init(argc, argv, "arm_camera_objects_recognition_server");
  ros::NodeHandle n;
  
  ros::ServiceServer service = n.advertiseService("acquire_image", acquire_image_cb);
  
 /* image_transport::ImageTransport image_transport(n);
  image_transport::CameraSubscriber image_sub = image_transport.subscribeCamera("/arm_camera/image_raw", 1, image_callback);*/
  
  client_hdr = n.serviceClient<pointgrey_camera_driver::HDRService>("hdr_camera_node");
  
  as=new actionlib::SimpleActionServer<youbot_object_recognition::RecognizeObjectAction>(n, "camera_arm_recognize_object", recognize_object_cb, false);
	as->start();
	
	ROS_INFO("camera_arm server ready");

  ros::spin();

  return 0;
}


