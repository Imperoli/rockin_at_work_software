#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>
#include "youbot_object_recognition/RecognizeObjectAction.h"
#include "std_msgs/String.h"
#include "mcr_perception_msgs/ObjectList.h"
#include "mcr_perception_msgs/PlanarPolygon.h"
#include <boost/thread.hpp>

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h> 
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

actionlib::SimpleActionServer<youbot_object_recognition::RecognizeObjectAction>* as;

ros::Publisher e_trigger_pub;

boost::mutex recognition_succeeded_mtx, object_list_acquired_mtx, ob_list_in_mtx, polygon_mtx, polygon_acquired_mtx;
mcr_perception_msgs::ObjectList ob_list_in;
mcr_perception_msgs::PlanarPolygon polygon;
bool object_list_acquired=false;
bool polygon_acquired=false;
int recognition_succeeded=2;

void polygon_cb(const mcr_perception_msgs::PlanarPolygon::ConstPtr& msg)
{
  polygon_mtx.lock();
  polygon_acquired_mtx.lock();
    polygon=*msg;
    polygon_acquired=true;
  polygon_acquired_mtx.unlock();
  polygon_mtx.unlock();
}

void rec_objects_cb(const mcr_perception_msgs::ObjectList::ConstPtr& msg)
{
  ob_list_in_mtx.lock();
  object_list_acquired_mtx.lock();
    ob_list_in=*msg;
    object_list_acquired=true;
  object_list_acquired_mtx.unlock();
  ob_list_in_mtx.unlock();
}

void rec_status_cb(const std_msgs::String::ConstPtr& msg)
{
  recognition_succeeded_mtx.lock();
    (msg->data.compare("e_done")==0)? recognition_succeeded=1:recognition_succeeded=0;
  recognition_succeeded_mtx.unlock();
}

void computePolygonCentroid(mcr_perception_msgs::PlanarPolygon &poly,geometry_msgs::Pose &centr)
{
  Eigen::Vector3f tot(0,0,0);
  for (int i=0; i<poly.contour.size(); i++)
  {
    Eigen::Vector3f p(poly.contour[i].x, poly.contour[i].y, poly.contour[i].z);
    tot=tot+p;
  }
  if (poly.contour.size()>0) tot=tot/poly.contour.size();
  
  centr.position.x=(double) tot(0);
  centr.position.y=(double) tot(1);
  centr.position.z=(double) tot(2);
}

void recognize_object_cb(const youbot_object_recognition::RecognizeObjectGoalConstPtr & goal){
  youbot_object_recognition::RecognizeObjectResult result;

  recognition_succeeded_mtx.lock();
    recognition_succeeded=2;
  recognition_succeeded_mtx.unlock();
  
  object_list_acquired_mtx.lock();
    object_list_acquired=false;
  object_list_acquired_mtx.unlock();
  
  polygon_acquired_mtx.lock();
    polygon_acquired=false;
  polygon_acquired_mtx.unlock();

	std_msgs::String e_in;
	e_in.data="e_trigger";
	e_trigger_pub.publish(e_in);
	
	recognition_succeeded_mtx.lock();
	while (recognition_succeeded==2)
	{
	  recognition_succeeded_mtx.unlock();
	  ROS_INFO("waiting for mcr_object_recognition result...");
	  ros::Rate r(2); r.sleep();
	  recognition_succeeded_mtx.lock();
	}
	recognition_succeeded_mtx.unlock();
	
	recognition_succeeded_mtx.lock();
	if(recognition_succeeded==0)
	{
	  recognition_succeeded_mtx.unlock();
	  as->setAborted(result);
	  return;
	}
	recognition_succeeded_mtx.unlock();
	
	object_list_acquired_mtx.lock();
	while (!object_list_acquired)
	{
	  object_list_acquired_mtx.unlock();
	  ros::Rate r(2); r.sleep();
	  ROS_INFO("waiting for object list...");
	  object_list_acquired_mtx.lock();
	}
	object_list_acquired_mtx.unlock();
	polygon_acquired_mtx.lock();
	while (!polygon_acquired)
	{
	  polygon_acquired_mtx.unlock();
	  ros::Rate r(2); r.sleep();
	  ROS_INFO("waiting for polygon...");
	  polygon_acquired_mtx.lock();
	}
	polygon_acquired_mtx.unlock();

	
	mcr_perception_msgs::ObjectList ob_list_out;
	ob_list_in_mtx.lock();
	polygon_mtx.lock();

	if(ob_list_in.objects.size()>0)
	{
	 for(size_t i=0; i<ob_list_in.objects.size(); i++)
	  {
	    if (ob_list_in.objects[i].name.compare(goal->object_name.data)==0||goal->object_name.data.compare("all")==0)
	    {
	      ob_list_out.objects.push_back(ob_list_in.objects[i]);
	    }
	  }
  }
  computePolygonCentroid(polygon,result.workspace_pose);

  polygon_mtx.unlock();
	ob_list_in_mtx.unlock();
	
	result.object_list=ob_list_out;
	as->setSucceeded(result);
}

int main(int argc, char **argv){

	ros::init(argc, argv, "kinect_object_recognition");
  	ros::NodeHandle nh;

	e_trigger_pub = nh.advertise<std_msgs::String>("/mcr_perception/object_detector/event_in", 1);
	ros::Subscriber rec_status_sub = nh.subscribe("/mcr_perception/object_detector/event_out", 1, rec_status_cb);
	ros::Subscriber rec_objects_sub = nh.subscribe("/mcr_perception/object_detector/object_list", 1, rec_objects_cb);
	ros::Subscriber polygon_sub = nh.subscribe("/mcr_perception/workspace_finder/polygon", 1, polygon_cb);

	as=new actionlib::SimpleActionServer<youbot_object_recognition::RecognizeObjectAction>(nh, "kinect_recognize_object", recognize_object_cb, false);
	as->start();

  ros::spin();

	return 1;
}
