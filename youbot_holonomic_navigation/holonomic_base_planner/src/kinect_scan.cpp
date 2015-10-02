#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/LaserScan.h"
#include "image_transport/image_transport.h"
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
//#include <eigen_conversions/eigen_msg.h>
#include "cv_bridge/cv_bridge.h"
#include <Eigen/Dense>

#include "opencv2/core/core.hpp"
#include <math.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

std::string virtual_laser_frame="virtual_laser";
std::string kinect_frame="tower_cam3d_depth_optical_frame";
float angle_min=-M_PI/2;
float angle_max=M_PI/2;
float angle_increment=0.008722;
float range_min=.01;
float range_max=5;

cv::Mat dbg_img;

ros::Publisher scan_pub;
Eigen::Affine3f T_kinect_virtual_laser;

float yawFromTarget(Eigen::Vector3f& target){
  if(fabs(target(0))<.00005&&target(1)>0) return 1.57079632679;
  else if(fabs(target(0))<.00005&&target(1)<0) return -1.57079632679;
  return atan2(target(1),target(0));
}

bool computeTransformation(std::string target, std::string source, Eigen::Affine3d& T)
{
  tf::TransformListener listener;
	tf::StampedTransform transform;
	for (int i=0; i<3;i++)
	{
	  try
    {
      ros::Time now=ros::Time::now();
      listener.waitForTransform( target, source, now, ros::Duration(1));
      listener.lookupTransform( target, source, now, transform);
      tf::transformTFToEigen(transform, T);
      return true;
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
    }
  }
  return false;
}

void depth_callback(const sensor_msgs::Image::ConstPtr& img, 
		   const sensor_msgs::CameraInfo::ConstPtr& info) {
		     
/*void depth_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{	   */
  sensor_msgs::LaserScan scan_out;
  scan_out.header.stamp=img->header.stamp;
  scan_out.header.frame_id=virtual_laser_frame;
  scan_out.angle_min=angle_min;
  scan_out.angle_max=angle_max;
  scan_out.range_min=range_min;
  scan_out.range_max=range_max;
  scan_out.angle_increment=angle_increment;
  scan_out.ranges.assign((int)round((angle_max-angle_min)/angle_increment),1000.0f);

  // Get camera info
  Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
  int i = 0;
  for(int r = 0; r < 3; r++) {
    for(int c = 0; c < 3; c++, i++) {
      K(r, c) = info->K[i];
    }
  }
  cv_bridge::CvImagePtr depth = cv_bridge::toCvCopy(img, img->encoding);
  //cv::cvtColor ( depth->image, dbg_img, cv::COLOR_GRAY2BGR );
  dbg_img=cv::Mat(depth->image.size(),CV_8UC3);
  for(int r = 0; r < depth->image.rows; r++) {
    for(int c = 0; c < depth->image.cols; c++) {
      // float d = float(depth->image.at<unsigned short int>(r, c)) * 0.001f;
       
      float d = depth->image.at<float>(r, c);
      dbg_img.at<cv::Vec3b>(r,c)=cv::Vec3b(d*255/3,d*255/3,d*255/3);
      //std::cout<<d<<std::endl;
      Eigen::Vector3f p;
      if(d < 4.0f && d > 0.1f) { p(2) = d; }
      else { continue;}
      
      p(0) = (float(c) - K(0, 2)) * p(2) / K(0, 0);
      p(1) = (float(r) - K(1, 2)) * p(2) / K(0, 0);		
      p=T_kinect_virtual_laser*p;
      //std::cout<<p.transpose()<<std::endl;
      if (p(0)<range_min||p(0)>range_max||p(2)<-.01||p(2)>.02) continue;
      
      float alfa=yawFromTarget(p)+1.5707;
      int index=round(alfa/angle_increment);
      float normp=p.norm();
      if (scan_out.ranges[index]>normp){
        scan_out.ranges[index]=normp;
        dbg_img.at<cv::Vec3b>(r,c)=cv::Vec3b(10,50, 200);
        //std::cout<<p(0)<<std::endl;
      }
    }
  }
  cv::resize(dbg_img,dbg_img,cv::Size(dbg_img.cols*2,dbg_img.rows*2));
  cv::imshow ( "depth", dbg_img );
  cv::waitKey(10);
  
  /*pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::fromROSMsg(*msg, cloud);

  for(int i=0;i<cloud.points.size();i++)
  {
    Eigen::Vector3f p(cloud.points[i].x,cloud.points[i].y,cloud.points[i].z);
    //std::cout<<p.transpose()<<std::endl;
    if(p(0)!=p(0)||p(2) > 4.0f || p(2) < 0.1f)
    { continue;}
    p=T_kinect_virtual_laser*p;
    //std::cout<<p.transpose()<<std::endl;
    if (p(0)<range_min||p(0)>range_max||p(2)<-.005||p(2)>.02) continue;
    
    float alfa=yawFromTarget(p)+1.5707;
    int index=round(alfa/angle_increment);
    if (scan_out.ranges[index]>p(0)){
      scan_out.ranges[index]=p(0);
      //std::cout<<p(0)<<std::endl;
    }
  }*/
  
  scan_pub.publish(scan_out);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "kinect_scan");
  ros::NodeHandle n;
  
  Eigen::Affine3d T_;
  computeTransformation(virtual_laser_frame, kinect_frame, T_);
  T_kinect_virtual_laser=Eigen::Affine3f(T_);

  image_transport::ImageTransport depth_image_transport(n);
  image_transport::CameraSubscriber depth_sub = depth_image_transport.subscribeCamera("/tower_cam3d/depth/image", 1, depth_callback);
  //ros::Subscriber sub = n.subscribe("/tower_cam3d/depth_registered/points", 1, depth_callback);
  
  scan_pub = n.advertise<sensor_msgs::LaserScan>("kinect_base_scan", 1);

  ros::spin();

  return 0;
}



