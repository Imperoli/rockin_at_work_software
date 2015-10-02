#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include "opencv2/core/core.hpp"
#include <math.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <boost/thread.hpp>
#include <Eigen/Dense>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "utils.h"

using namespace cv;

Mat scan_image;
Mat map_image;
Mat show;
Mat distance_map, map;
boost::mutex mtx_scan_image;

ros::Publisher pub_vel;
actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction>* as;

mapInfo map_info;
robotInfo robot_info;

void laser_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	float angle_min=msg->angle_min;
	float angle_incr=msg->angle_increment;
	float range_scan_min=msg->range_min;
	float range_scan_max=msg->range_max;
	std::vector<float> ranges;
	ranges=msg->ranges;
	
	float h=(float)map_info.im_size;
	float w=h;
	
	mtx_scan_image.lock();
	scan_image=Scalar(0);

  for(size_t i = 0; i < ranges.size(); i++){
		if(ranges[i]>range_scan_min&&ranges[i]<range_scan_max){
			float posx=ranges[i]*cos(angle_min+((float)(i)*angle_incr))+robot_info.laser_x;
			float posy=ranges[i]*sin(angle_min+((float)(i)*angle_incr));
			
			float indy=-((posy/map_info.resolution)-h/2);
			indy=h-indy;

			float indx=(-(posx/map_info.resolution)+w/2);
			if(indy>=0&&indy<map_info.im_size&&indx>=0&&indx<map_info.im_size)
			  scan_image.at<unsigned char>((int)round(indy),(int)round(indx))=255;
    }
	}
	mtx_scan_image.unlock();
}

void computeRepulsionFromImages(std::vector<Eigen::Vector2f>& repulsion_v)
{
  repulsion_v.clear();
  float min_dist=1000;
  mtx_scan_image.lock();
  
  show=Scalar(0);
  std::vector<Point> v;
  
  
  for(int i=0;i<map_info.im_size;i++){
    for(int j=0; j<map_info.im_size; j++){
      if((int)scan_image.at<unsigned char>(i,j)==255||(int)map_image.at<unsigned char>(i,j)==255)
      {
        if((int)scan_image.at<unsigned char>(i,j)==255) show.at<Vec3b>(j,map_info.im_size-i-1)[2]=255;
        if((int)map_image.at<unsigned char>(i,j)==255) show.at<Vec3b>(j,map_info.im_size-i-1)[1]=255;
        if(min_dist>distance_map.at<Vec3f>(i,j)[2])
        {
          min_dist=distance_map.at<Vec3f>(i,j)[2];
         /* iok=i; jok=j;
          if(distance_map.at<Vec3f>(i,j)[2]>.00001)
          {
            repulsion(0)=distance_map.at<Vec3f>(i,j)[0]/distance_map.at<Vec3f>(i,j)[2];
            repulsion(1)=distance_map.at<Vec3f>(i,j)[1]/distance_map.at<Vec3f>(i,j)[2];
          }
          else
          {
            repulsion(0)=distance_map.at<Vec3f>(i,j)[0]/.00001;
            repulsion(1)=distance_map.at<Vec3f>(i,j)[1]/.00001;
          }*/
        }
      }
    }
  }
  
   for(int i=0;i<map_info.im_size;i++){
    for(int j=0; j<map_info.im_size; j++){
      if((int)scan_image.at<unsigned char>(i,j)==255||(int)map_image.at<unsigned char>(i,j)==255)
      {
        if(min_dist==distance_map.at<Vec3f>(i,j)[2])
        {
          v.push_back(Point(j,i));
          Eigen::Vector2f rep(distance_map.at<Vec3f>(i,j)[0]/distance_map.at<Vec3f>(i,j)[2], distance_map.at<Vec3f>(i,j)[1]/distance_map.at<Vec3f>(i,j)[2]);
          repulsion_v.push_back(rep);
        }
      }
    }
   }
  
  for(size_t i=0;i<v.size();i++){
    show.at<Vec3b>(v[i].x, map_info.im_size-v[i].y-1)=Vec3b(255,255,255);
  }
  
  Mat show2;
  resize(show, show2, cv::Size(400,400), 0, 0,cv::INTER_NEAREST);
  
  cv::imshow("local_map",show2);
	cv::waitKey(1);
	
  mtx_scan_image.unlock();
}

void computeMapImage(float robot_posx, float robot_posy, float robot_orient)
{
  float r_x_px=(robot_posx-map_info.map_origin_x)/map_info.resolution; float r_y_px=(float)map.rows-1+((map_info.map_origin_y-robot_posy)/map_info.resolution);
  map_image=Scalar(0);
  Rect rect;
  float roi_size=(float)map_info.im_size*1;
  rect = Rect((int)round(r_x_px-roi_size/2), (int)round(r_y_px-roi_size/2), (int)roi_size, (int)roi_size);
  Mat roi_image;
  if(rect.x<0||rect.x>=map.cols-rect.width||rect.y<0||rect.y>=map.rows-rect.height){return;}
  roi_image = map(rect);
  Point p0(0,0);
  for(int i=0;i<roi_size;i++){
    for(int j=0;j<roi_size;j++){
      Point idx(j-roi_size/2,i-roi_size/2);
      Point p=p0+idx;
      if((int)roi_image.at<unsigned char>(i,j)<map_info.map_treshold)
      {
        p.y=-p.y; p.x=-p.x;
        float j_pos=(float)p.x*cos(robot_orient)-(float)p.y*sin(robot_orient)+(float)roi_size/2;
        float i_pos=(float)p.x*sin(robot_orient)+(float)p.y*cos(robot_orient)+(float)roi_size/2;
        if((int)round(i_pos)<map_image.rows&&i_pos>=0&&(int)round(j_pos)<map_image.cols&&j_pos>=0   &&  j_pos>roi_size/3)
        {
          map_image.at<unsigned char>((int)round(i_pos), (int)round(j_pos))=255;
        }
      }
    }
  }
}

void action_cb(const move_base_msgs::MoveBaseGoalConstPtr& goal)
{
  tf::TransformListener listener;
	std::string base_frame="base_footprint";
	//std::string map_frame="map";
	std::string map_frame=goal->target_pose.header.frame_id;
	tf::StampedTransform transform;
	tf::Vector3 axis;
  ros::Rate lr(10);
  Eigen::Vector2f current_vel(0,0);
  
  Eigen::Vector2f target(goal->target_pose.pose.position.x, goal->target_pose.pose.position.y);
  
  float target_theta;
  Eigen::Quaternionf q(goal->target_pose.pose.orientation.w, goal->target_pose.pose.orientation.x, goal->target_pose.pose.orientation.y, goal->target_pose.pose.orientation.z);
  Eigen::Matrix3f m(q);
  target_theta=atan2(m(1,0),m(0,0));
  target_theta=(target_theta<0)? target_theta+2*M_PI : (target_theta>2*M_PI)? target_theta-2*M_PI : target_theta;
  
  std::vector<Eigen::Vector2f> repulsion_vector;
  
  ros::Time init_time(ros::Time::now());
  ros::Time current_time(ros::Time::now());
  
  while(ros::ok()&&(current_time-init_time)<ros::Duration(60.0))
  {
    try{
      ros::Time now = ros::Time::now();
      listener.waitForTransform(map_frame, base_frame, now, ros::Duration(1));
      listener.lookupTransform(map_frame, base_frame, now, transform);
      
      float robot_posx=transform.getOrigin().x();
	    float robot_posy=transform.getOrigin().y();
	    float robot_orient=transform.getRotation().getAngle();
	    axis=transform.getRotation().getAxis();		
	    robot_orient*=axis[2];
	    robot_orient=(robot_orient<0)? robot_orient+2*M_PI : (robot_orient>2*M_PI)? robot_orient-2*M_PI : robot_orient;  
	    if((robot_orient-target_theta)>M_PI) robot_orient-=2*M_PI;
	    else if((robot_orient-target_theta)<-M_PI) robot_orient+=2*M_PI;
	    
	    // 19.7 28.2
	    // 2.17 4.88
	    // -.27, -.94
      // -2.27, -.4
      
      //-2.17177391052, -0.441132843494
      //-1.01823496819, -0.79
      //0.536493480206, 1.77734947205
      //2.64069199562, 0.474561214447
      //2.07868504524, -0.241159290075

      Eigen::Vector2f attraction,vel, robot_pos;

      robot_pos=Eigen::Vector2f(robot_posx,robot_posy);
      
      geometry_msgs::Twist cmd_vel;
     // if ((robot_pos-target).norm()>.05){

        computeMapImage(robot_posx, robot_posy, robot_orient);
        computeRepulsionFromImages(repulsion_vector);
        
        attraction=target-robot_pos;
        float attr_norm=attraction.norm(); float k=attr_norm;
        attraction=(attraction/k);
        k=(k>.3)? k=.3:(k<.1)? k=.1: k;
        attraction*=k;
        //repulsion=(attr_norm>.15)? repulsion*(k/10): repulsion*(k/20);
        float sn=sin(robot_orient); float cs=cos(robot_orient);
        
        Eigen::Vector2f attr(attraction);
        for (size_t i=0;i<repulsion_vector.size(); i++)
        {
          repulsion_vector[i]=(attr_norm>.15)? repulsion_vector[i]*(k/10): repulsion_vector[i]*(k/20);
          float r0=repulsion_vector[i](0); float r1=repulsion_vector[i](1);
          repulsion_vector[i](0)=r0*cs-r1*sn;
		      repulsion_vector[i](1)=r0*sn+r1*cs;
		      Eigen::Vector2f ar=attraction+repulsion_vector[i];
		      (ar(0)*attr(0)<0)? attr(0)=0:(fabs(ar(0))>fabs(attr(0)))? attr(0): attr(0)=ar(0);
		      (ar(1)*attr(1)<0)? attr(1)=0:(fabs(ar(1))>fabs(attr(1)))? attr(1): attr(1)=ar(1);;
		    }
		    attraction=attr;
		    
        float a0=attraction(0); float a1=attraction(1);
        attraction(0)=a0*cs+a1*sn;
		    attraction(1)=-a0*sn+a1*cs;
		    
		    if(attr_norm<=.05) attraction=Eigen::Vector2f(0,0);
        //vel=attraction+repulsion;
        vel=attraction;
        /*std::cout<<"pos\n"<<robot_pos<<std::endl;
        std::cout<<"repulsion\n"<<repulsion<<std::endl;
        std::cout<<"attraction\n"<<attraction<<std::endl;
        std::cout<<"vel\n"<<vel<<std::endl;*/

        for(int i=0;i<=1;i++){
          if (fabs(vel(i))<0.01){ current_vel(i)=vel(i); continue;}
          if (vel(i) - current_vel(i) > 0.05) current_vel(i) +=0.05;
          else if(vel(i) - current_vel(i) < -0.05) current_vel(i) -=0.05;
          else current_vel(i) = vel(i);
        }
        //current_vel = vel;
        cmd_vel.linear.x = current_vel(0);
        cmd_vel.linear.y = current_vel(1);
        
     // }
     // else
     // {
        
     //   cmd_vel.linear.x = 0;
     //   cmd_vel.linear.y = 0;
     //   current_vel=Eigen::Vector2f(0,0);
       
     // }
     
     // std::cout<<" theta "<< robot_orient<< " "<<target_theta<<"\n"<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<std::endl;
      if((robot_pos-target).norm()<=.11)
      {
        float theta_diff=fabs(robot_orient-target_theta);
        theta_diff=(theta_diff>.4)? .4:(theta_diff<.005)? .005:theta_diff;
        cmd_vel.angular.z=((robot_orient-target_theta)>.005)? cmd_vel.angular.z-theta_diff: ((robot_orient-target_theta)<.005)? cmd_vel.angular.z+theta_diff: cmd_vel.angular.z;
      }
      
      if((robot_pos-target).norm()<=.05 && (fabs(robot_orient-target_theta)<=.005))
      {
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.angular.z=0;
        current_vel=Eigen::Vector2f(0,0);
        pub_vel.publish(cmd_vel);
        as->setSucceeded();
        return;
      }
      
      pub_vel.publish(cmd_vel);
    }
    catch (tf::TransformException ex){
    //  ROS_ERROR("holonomic_base_planner_main: %s",ex.what());
    //  ROS_ERROR_STREAM("TF from " << base_frame << "  to " << map_frame);
    }
    lr.sleep();
    current_time=ros::Time::now();
  }
  
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0;
  cmd_vel.linear.y = 0;
  cmd_vel.angular.z=0;
  pub_vel.publish(cmd_vel);
  as->setAborted();
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "holonomic_base_planner_main");
  ros::NodeHandle n;
  
  readInfoFiles(argv[1],map_info,robot_info);
  scan_image=Mat(map_info.im_size,map_info.im_size,CV_8UC1);
  map_image=Mat(map_info.im_size,map_info.im_size,CV_8UC1);
  show=Mat(map_info.im_size,map_info.im_size,CV_8UC3);
  
  pub_vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Subscriber sub_laser = n.subscribe("base_scan", 1, laser_cb);
  
  mtx_scan_image.lock();
	scan_image=Scalar(0);
	mtx_scan_image.unlock();
	
	compute_distance_map(map_info,robot_info,distance_map);
  
  map = imread(map_info.file_name, CV_LOAD_IMAGE_GRAYSCALE);
  map.at<unsigned char>(map.rows-1+map_info.map_origin_y/map_info.resolution,-map_info.map_origin_x/map_info.resolution)=255;

  map_image=Scalar(0);
  
  as =new actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction>(n, "holonomic_goto", action_cb, false);
  as->start();
  
  ros::spin();
  
  return 0; 
}

