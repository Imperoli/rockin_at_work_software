#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>
#include "youbot_object_recognition/RecognizeObjectAction.h"
#include "mcr_perception_msgs/ObjectList.h"
#include <actionlib/client/simple_action_client.h>
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "std_srvs/Empty.h"

#define N_PROC 1

std::vector<actionlib::SimpleActionClient<youbot_object_recognition::RecognizeObjectAction>*> ac_processes;
actionlib::SimpleActionServer<youbot_object_recognition::RecognizeObjectAction>* as;
boost::mutex image_mtx;
bool acquire_image=false;
bool image_acquired=false;
sensor_msgs::Image image;
sensor_msgs::CameraInfo camera_info;

bool acquire_image_cb(std_srvs::Empty::Request  &req,
         std_srvs::Empty::Response &res)
{
  image_mtx.lock();
  acquire_image=true;
  image_mtx.unlock();
  return true;
}

void image_callback(const sensor_msgs::Image::ConstPtr& img, 
		   const sensor_msgs::CameraInfo::ConstPtr& info) {  

  
  image_mtx.lock();
  if(acquire_image)
  {
    image=*img;
    camera_info=*info;

  }
  acquire_image=false;
  image_acquired=true;
  image_mtx.unlock();
}

void recognize_object_cb(const youbot_object_recognition::RecognizeObjectGoalConstPtr & goal)
{
  youbot_object_recognition::RecognizeObjectGoal splitted_goal=*goal;
  image_mtx.lock();
  splitted_goal.image=image;
  splitted_goal.camera_info=camera_info;
  image_mtx.unlock();
  int init_guess_size=goal->initial_guess_list.size();
  int k=0; int count=0;
  for (int i=0; i<N_PROC; i++)
  {
    splitted_goal.initial_guess_list.clear();
    //std::cout<<"init guess size: "<<init_guess_size<<std::endl;
    
    for(;k<count+init_guess_size/N_PROC&&k<init_guess_size; k++)
    {
      splitted_goal.initial_guess_list.push_back(goal->initial_guess_list[k]);
     // std::cout<<"init guess size: "<<init_guess_size<<" k "<<k<<std::endl;
    }
    count=k;
    std::string server_name="camera_arm_recognize_object_single_process_";
    std::stringstream ss; ss<<server_name<<i;
   // std::cout<<"sending goal to "<<ss.str()<<std::endl;
    ac_processes[i]->sendGoal(splitted_goal);
  }
  
  youbot_object_recognition::RecognizeObjectResult result;
  
  float max_probability=0;
  for (int i=0; i<N_PROC; i++)
  {
    ac_processes[i]->waitForResult();
    if(ac_processes[i]->getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      youbot_object_recognition::RecognizeObjectResultConstPtr camera_arm_result;
	    camera_arm_result=ac_processes[i]->getResult();
	    if(camera_arm_result->object_list.objects[0].probability<=max_probability||camera_arm_result->object_list.objects[0].probability<.5) 
	    {
	      ROS_INFO("too low score");
        continue;
	    }
	    max_probability=camera_arm_result->object_list.objects[0].probability;
	    result.object_list.objects.clear();
      result.object_list.objects.push_back(camera_arm_result->object_list.objects[0]);
    }
    else
    {
      ROS_INFO("camera_arm recognition failed");
      continue;
    }
  }
  if(result.object_list.objects.size()>0)
    as->setSucceeded(result);
  else
    as->setAborted(result);
}

int main ( int argc, char** argv )
{ 
  ros::init(argc, argv, "arm_camera_objects_recognition_server");
  ros::NodeHandle n;
  
  ros::ServiceServer service = n.advertiseService("acquire_image", acquire_image_cb);
  
  image_transport::ImageTransport image_transport(n);
  image_transport::CameraSubscriber image_sub = image_transport.subscribeCamera("/arm_camera/image_raw", 1, image_callback);
  
  for (int i=0; i<N_PROC; i++)
  {
    std::string server_name="camera_arm_recognize_object_single_process_";
    std::stringstream ss; ss<<server_name<<i;
    actionlib::SimpleActionClient<youbot_object_recognition::RecognizeObjectAction>* ac=new actionlib::SimpleActionClient<youbot_object_recognition::RecognizeObjectAction>(ss.str(), true);
    ac_processes.push_back(ac);
    std::cout<<"waiting for "<<ss.str()<<std::endl;
    ac_processes[i]->waitForServer();
  }

  as=new actionlib::SimpleActionServer<youbot_object_recognition::RecognizeObjectAction>(n, "camera_arm_recognize_object", recognize_object_cb, false);
	as->start();
	
	ROS_INFO("camera_arm server ready");

  ros::spin();

  return 0;
}


