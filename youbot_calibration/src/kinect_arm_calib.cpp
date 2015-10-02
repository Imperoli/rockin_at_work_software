#include <ros/ros.h>
#include <math.h>
#include <Eigen/Dense>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <fstream>


int main(int argc, char **argv)
{

	ros::init(argc, argv, "kinect_arm_calib");

	ros::NodeHandle n;
  tf::TransformListener listener;
	tf::StampedTransform transform;

  Eigen::Affine3d T_cam;
  Eigen::Affine3d T_arm;

  char cmd;
  
  std::cout<<"type for computing frame in camera_rgb_optical_frame"<<std::endl;
  std::cin>>cmd;
  Eigen::Vector3d ar0_cam(0,0,0); Eigen::Vector3d ar1_cam(0,0,0); Eigen::Vector3d ar3_cam(0,0,0);
  int count=0;
 // for(int i=0;i<5;i++)
 // {
    try
    {
      for(int i=0;i<30;i++)
      {
          ros::Time now=ros::Time::now();
          listener.waitForTransform( "camera_rgb_optical_frame", "ar_marker_0", now, ros::Duration(1));
          listener.lookupTransform( "camera_rgb_optical_frame", "ar_marker_0", now, transform);
          ar0_cam+=Eigen::Vector3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
          
          listener.waitForTransform( "camera_rgb_optical_frame", "ar_marker_1", now, ros::Duration(1));
          listener.lookupTransform( "camera_rgb_optical_frame", "ar_marker_1", now, transform);
          ar1_cam+=Eigen::Vector3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
          
          listener.waitForTransform( "camera_rgb_optical_frame", "ar_marker_3", now, ros::Duration(1));
          listener.lookupTransform( "camera_rgb_optical_frame", "ar_marker_3", now, transform);
          ar3_cam+=Eigen::Vector3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
          count++;
      }
      //break;
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      return -1;
    }
  //}

  if (count>0)
  {
    ar0_cam/=count; ar1_cam/=count; ar3_cam/=count;
  }
  else return -1;
  
  Eigen::Vector3d x_cam=(ar1_cam-ar0_cam)/(ar1_cam-ar0_cam).norm();
  Eigen::Vector3d y_cam=(ar3_cam-ar0_cam)/(ar3_cam-ar0_cam).norm();
  Eigen::Vector3d z_cam=x_cam.cross(y_cam);
  
  Eigen::Matrix3d R_cam;
  R_cam.col(0)=x_cam; R_cam.col(1)=y_cam; R_cam.col(2)=z_cam;
  T_cam.linear()=R_cam; T_cam.translation()=ar0_cam;
  std::cout<<"T CAM\n"<<T_cam.matrix()<<std::endl;
  
  Eigen::Vector3d ar0_arm; Eigen::Vector3d ar1_arm; Eigen::Vector3d ar3_arm;
  
  std::cout<<"type for acquiring ar_marker_0 in arm_base frame"<<std::endl;
  std::cin>>cmd;
  try
  {
    ros::Time now=ros::Time::now();
    listener.waitForTransform( "arm_base", "end_effector_edge", now, ros::Duration(1));
    listener.lookupTransform( "arm_base", "end_effector_edge", now, transform);
    ar0_arm=Eigen::Vector3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
   // ar0_arm=Eigen::Vector3d(-.1864, -.3421, -.085);
   return -1;
  }
  
  std::cout<<"type for acquiring ar_marker_1 in arm_base frame"<<std::endl;
  std::cin>>cmd;
  try
  {
    ros::Time now=ros::Time::now();
    listener.waitForTransform( "arm_base", "end_effector_edge", now, ros::Duration(1));
    listener.lookupTransform( "arm_base", "end_effector_edge", now, transform);
    ar1_arm=Eigen::Vector3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    //ar1_arm=Eigen::Vector3d(-.1139, -.34, -.085);
    return -1;
  }
  
  std::cout<<"type for acquiring ar_marker_3 in arm_base frame"<<std::endl;
  std::cin>>cmd;
  try
  {
    ros::Time now=ros::Time::now();
    listener.waitForTransform( "arm_base", "end_effector_edge", now, ros::Duration(1));
    listener.lookupTransform( "arm_base", "end_effector_edge", now, transform);
    ar3_arm=Eigen::Vector3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    //ar3_arm=Eigen::Vector3d(-.18723, -.4159, -.085);
    return -1;
  }

  Eigen::Vector3d x_arm=(ar1_arm-ar0_arm)/(ar1_arm-ar0_arm).norm();
  Eigen::Vector3d y_arm=(ar3_arm-ar0_arm)/(ar3_arm-ar0_arm).norm();
  Eigen::Vector3d z_arm=x_arm.cross(y_arm);
  
  Eigen::Matrix3d R_arm;
  R_arm.col(0)=x_arm; R_arm.col(1)=y_arm; R_arm.col(2)=z_arm;
  T_arm.linear()=R_arm; T_arm.translation()=ar0_arm;
  std::cout<<"T ARM\n"<<T_arm.matrix()<<std::endl;
  
  Eigen::Affine3d T_calib(T_arm*T_cam.inverse());  
	try{
	  ros::Time now=ros::Time::now();
    listener.waitForTransform(  "camera_rgb_optical_frame", "camera_link", now, ros::Duration(1));
    listener.lookupTransform(  "camera_rgb_optical_frame", "camera_link", now, transform);
    Eigen::Affine3d T;
    tf::transformTFToEigen(transform, T);
    Eigen::Affine3d T_calib_tot(T_calib*T);
    
    std::cout<<"T CAM_ARM TOT\n"<<(T_calib_tot).matrix()<<std::endl;
    Eigen::Quaterniond q(T_calib_tot.rotation());
    std::cout<<T_calib_tot.matrix().col(3)(0)<<" "<<T_calib_tot.matrix().col(3)(1)<<" "<<T_calib_tot.matrix().col(3)(2)<<" "<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<std::endl;
    std::ofstream of(argv[1]);of << T_calib_tot.matrix()<<"\n"<<T_calib_tot.matrix().col(3)(0)<<" "<<T_calib_tot.matrix().col(3)(1)<<" "<<T_calib_tot.matrix().col(3)(2)<<" "<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<std::endl;
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    return -1;
  }
  
	return 0;
}

