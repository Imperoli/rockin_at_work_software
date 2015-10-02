#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>
#include "youbot_object_recognition/RecognizeObjectAction.h"
#include "std_msgs/String.h"
#include "mcr_perception_msgs/ObjectList.h"
#include <boost/thread.hpp>

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h> 
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

actionlib::SimpleActionServer<youbot_object_recognition::RecognizeObjectAction>* as;

ros::Publisher e_trigger_pub;

boost::mutex recognition_succeeded_mtx, object_list_acquired_mtx, ob_list_in_mtx, clusters_mtx, clusters_acquired_mtx;
mcr_perception_msgs::ObjectList ob_list_in;
std::vector<geometry_msgs::Point> positions;
bool object_list_acquired=false; bool clusters_acquired=false;
int recognition_succeeded=2;

void clusters_cb(const sensor_msgs::PointCloud2::Ptr& msg)
{
  clusters_mtx.lock();
  clusters_acquired_mtx.lock();
  
  positions.clear();
  
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
  
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new  pcl::PointCloud<pcl::PointXYZ>);
  //pcl::fromPCLPointCloud2(*cloud_filtered, voxels);
  
  pcl_conversions::toPCL(*msg, *cloud);
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.045f, 0.045f, 0.045f);
  sor.filter (*cloud_filtered);
  
  pcl::PointCloud<pcl::PointXYZ> voxels;
  pcl::fromPCLPointCloud2(*cloud_filtered, voxels);
  for (int i=0;i<voxels.points.size();i++)
  {
    geometry_msgs::Point p;
    p.x=voxels.points[i].x;
    p.y=voxels.points[i].y;
    p.z=voxels.points[i].z;
    positions.push_back(p);
  }
  
  clusters_acquired=true;
  clusters_acquired_mtx.unlock();
  clusters_mtx.unlock();
}

void computeMultipleInitialGuess(mcr_perception_msgs::Object& ob_in, mcr_perception_msgs::ObjectList& ob_list)
{
  ob_list.objects.clear();
  geometry_msgs::PoseStamped pos;
  pos=ob_in.pose;
  
  for (int i=0; i<positions.size(); i++)
  {
    mcr_perception_msgs::Object ob;
    pos.pose.position=positions[i];
    ob.pose=pos;
    ob_list.objects.push_back(ob);
  }
  
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

void recognize_object_cb(const youbot_object_recognition::RecognizeObjectGoalConstPtr & goal){
  youbot_object_recognition::RecognizeObjectResult result;

  recognition_succeeded_mtx.lock();
    recognition_succeeded=2;
  recognition_succeeded_mtx.unlock();
  
  object_list_acquired_mtx.lock();
    object_list_acquired=false;
  object_list_acquired_mtx.unlock();
  clusters_acquired_mtx.lock();
    clusters_acquired=false;
  clusters_acquired_mtx.unlock();

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
	clusters_acquired_mtx.lock();
	while (!clusters_acquired)
	{
	  clusters_acquired_mtx.unlock();
	  ros::Rate r(2); r.sleep();
	  ROS_INFO("waiting for clusters...");
	  clusters_acquired_mtx.lock();
	}
	clusters_acquired_mtx.unlock();
	
	mcr_perception_msgs::ObjectList ob_list_out;
	ob_list_in_mtx.lock();
	clusters_mtx.lock();
	if(ob_list_in.objects.size()>0)
	{
	  ob_list_out.objects.push_back(ob_list_in.objects[0]);
	/*  mcr_perception_msgs::ObjectList ob_l;
    computeMultipleInitialGuess(ob_list_in.objects[0], ob_l);
    for(int k=0;k<ob_l.objects.size();k++)
    {
      ob_list_out.objects.push_back(ob_l.objects[k]);
      
     
    }*/
  }
	/*for(size_t i=0; i<ob_list_in.objects.size(); i++)
	{
	  if (ob_list_in.objects[i].name.compare(goal->object_name.data)==0||goal->object_name.data.compare("all")==0)
	  {
	    ob_list_out.objects.push_back(ob_list_in.objects[i]);
	    
        
	  }
	}*/
	
	
	
	//computeMultipleInitialGuess(ob_list_in.objects[0], ob_list_out);
	
	clusters_mtx.unlock();
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
	ros::Subscriber clusters_sub = nh.subscribe("/mcr_perception/tabletop_cloud_accumulator/accumulated_cloud", 1, clusters_cb);


	as=new actionlib::SimpleActionServer<youbot_object_recognition::RecognizeObjectAction>(nh, "kinect_recognize_object", recognize_object_cb, false);
	as->start();

  ros::spin();

	return 1;
}
