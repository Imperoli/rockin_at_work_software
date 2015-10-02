#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
//#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Dense>
#include <boost/thread.hpp>

std::string virtual_laser_frame="virtual_laser";
std::string laser_frame="hokuyo";

ros::Publisher scan_pub;
Eigen::Affine3f T_laser_virtual_laser;
sensor_msgs::LaserScan virtual_scan;
boost::mutex virtual_mtx;

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

void virtual_cb(const sensor_msgs::LaserScan::ConstPtr& msg) {  
  virtual_mtx.lock();
  virtual_scan=*msg;
  virtual_mtx.unlock();
}

void laser_cb(const sensor_msgs::LaserScan::ConstPtr& msg) {  
  virtual_mtx.lock();
  if(fabs((virtual_scan.header.stamp-msg->header.stamp).toSec())>.1){virtual_mtx.unlock(); scan_pub.publish(*msg); return;}
  
  sensor_msgs::LaserScan scan_out;
  scan_out.header=msg->header;
  float angle_max=msg->angle_max;
  scan_out.angle_max=angle_max;
  float angle_min=-M_PI;
  scan_out.angle_min=angle_min;
  float angle_increment=msg->angle_increment;
  scan_out.angle_increment=angle_increment;
  float range_max=msg->range_max;
  scan_out.range_max=range_max;
  float range_min=msg->range_min;
  scan_out.range_min=range_min;
  scan_out.ranges.assign((int)round((angle_max-angle_min)/angle_increment),1000.0f);
  
  int index_offset=(int)round((msg->angle_min-angle_min)/angle_increment);
  for(int i=0;i<msg->ranges.size();i++)
  {
    int index=i+index_offset;
    scan_out.ranges[index]=msg->ranges[i];
  }
  for(int i=0;i<virtual_scan.ranges.size();i++)
  {
    if(virtual_scan.ranges[i]>virtual_scan.range_min&&virtual_scan.ranges[i]<virtual_scan.range_max)
    {
      Eigen::Vector3f p(virtual_scan.ranges[i]*cos(virtual_scan.angle_min+((float)(i)*virtual_scan.angle_increment)), virtual_scan.ranges[i]*sin(virtual_scan.angle_min+((float)(i)*virtual_scan.angle_increment)), 0);
      p=T_laser_virtual_laser*p;
      float alfa=yawFromTarget(p);
      int index=round(alfa/scan_out.angle_increment);
      if (scan_out.ranges[index]>p(0)){
        scan_out.ranges[index]=p(0);
      }
    }
  }
  virtual_mtx.unlock();
  scan_pub.publish(scan_out);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "multiple_scan_merger");
  ros::NodeHandle n;
  
  Eigen::Affine3d T_;
  computeTransformation(virtual_laser_frame, laser_frame, T_);
  T_laser_virtual_laser=Eigen::Affine3f(T_);

  ros::Subscriber laser_sub=n.subscribe("base_scan", 1, laser_cb);
  ros::Subscriber virtual_sub=n.subscribe("kinect_base_scan", 1, virtual_cb);
  scan_pub = n.advertise<sensor_msgs::LaserScan>("merged_base_scan", 1);

  ros::spin();

  return 0;
}



