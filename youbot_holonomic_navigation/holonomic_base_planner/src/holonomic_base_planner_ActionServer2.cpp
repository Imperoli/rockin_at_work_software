#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include "opencv2/core/core.hpp"
#include <math.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <boost/thread.hpp>
#include "Eigen/Dense"
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

using namespace cv;

float resolution=0.05;
float radius=.8;
float base_x=0.65;
float base_y=.4;
int im_size=round(radius*2/resolution);
int map_treshold=135;
float map_origin_x=-7.15; float map_origin_y=-3.8;

Mat scan_image(im_size,im_size,CV_8UC1);
Mat map_image(im_size,im_size,CV_8UC1);
Mat show(im_size,im_size,CV_8UC3);
Mat distance_map, map;
boost::mutex mtx_scan_image;

ros::Publisher pub_vel;
actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction>* as;

void laser_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	float angle_min=msg->angle_min;
	float angle_incr=msg->angle_increment;
	float range_scan_min=msg->range_min;
	float range_scan_max=msg->range_max;
	std::vector<float> ranges;
	ranges=msg->ranges;
	
	float h=(float)im_size;
	float w=h;
	
	mtx_scan_image.lock();
	scan_image=Scalar(0);

  for(size_t i = 0; i < ranges.size(); i++){
		if(ranges[i]>range_scan_min&&ranges[i]<range_scan_max){
			float posx=ranges[i]*cos(angle_min+((float)(i)*angle_incr))+.31;
			float posy=ranges[i]*sin(angle_min+((float)(i)*angle_incr));
			
			float indy=-((posy/resolution)-h/2);
			indy=h-indy;

			float indx=(-(posx/resolution)+w/2);
			if(indy>=0&&indy<im_size&&indx>=0&&indx<im_size)
			  scan_image.at<unsigned char>((int)round(indy),(int)round(indx))=255;
    }
	}
	mtx_scan_image.unlock();
}

void computeRepulsionFromImages(Eigen::Vector2f& repulsion)
{
  repulsion=Eigen::Vector2f(0,0);
  
  std::vector<Eigen::Vector3f> v;
  for (size_t i=0; i<4; i++){
    Eigen::Vector3f p(0,0,0);
    v.push_back(p);
  }
  
  mtx_scan_image.lock();
  
  show=Scalar(0);
  
  for(int i=0;i<im_size;i++){
    for(int j=0; j<im_size; j++){
      if((int)scan_image.at<unsigned char>(i,j)==255||(int)map_image.at<unsigned char>(i,j)==255)
      {
        if((int)scan_image.at<unsigned char>(i,j)==255) show.at<Vec3b>(i,j)[2]=255;
        if((int)map_image.at<unsigned char>(i,j)==255) show.at<Vec3b>(i,j)[1]=255;
        int index=(int)distance_map.at<Vec3f>(i,j)[2];
        float dist=Eigen::Vector2f(distance_map.at<Vec3f>(i,j)[0],distance_map.at<Vec3f>(i,j)[1]).norm();
        if(v[index](2)<dist)
        {
          v[index](0)=distance_map.at<Vec3f>(i,j)[0];
          v[index](1)=distance_map.at<Vec3f>(i,j)[1];
          v[index](2)=dist;
        }
      }
    }
  }
  float count=0;
  for (size_t i=0; i<4; i++){
    if (v[i](2)>0)
    {
      repulsion+=v[i].head(2)*v[i](2);
      count+=v[i](2);
    }
  }
  (count>0)? repulsion=repulsion/count:repulsion;
  
  //show.at<Vec3b>(iok,jok)=Vec3b(255,255,255);
  Mat show2;
  resize(show, show2, cv::Size(400,400), 0, 0,cv::INTER_NEAREST);
  cv::imshow("images",show2);
	cv::waitKey(1);
	
  mtx_scan_image.unlock();
}

void computeMapImage(float robot_posx, float robot_posy, float robot_orient)
{
  float r_x_px=(robot_posx-map_origin_x)/resolution; float r_y_px=(float)map.rows-1+((map_origin_y-robot_posy)/resolution);
  map_image=Scalar(0);
  
  Rect rect;
  float roi_size=(float)im_size*1;
  rect = Rect((int)round(r_x_px-roi_size/2), (int)round(r_y_px-roi_size/2), (int)roi_size, (int)roi_size);
  Mat roi_image;
  if(rect.x<0||rect.x>=map.cols-rect.width||rect.y<0||rect.y>=map.rows-rect.height){return;}
  roi_image = map(rect);
  
  Point p0(0,0);
  for(int i=0;i<roi_size;i++){
    for(int j=0;j<roi_size;j++){
      Point idx(j-roi_size/2,i-roi_size/2);
      Point p=p0+idx;
      if((int)roi_image.at<unsigned char>(i,j)<map_treshold)
      {
        p.y=-p.y; p.x=-p.x;
        float j_pos=(float)p.x*cos(robot_orient)-(float)p.y*sin(robot_orient)+(float)roi_size/2;
        float i_pos=(float)p.x*sin(robot_orient)+(float)p.y*cos(robot_orient)+(float)roi_size/2;
        if((int)round(i_pos)<map_image.rows&&i_pos>=0&&(int)round(j_pos)<map_image.cols&&(j_pos)>=0     &&  j_pos>roi_size/3)
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
	std::string map_frame="map";
	tf::StampedTransform transform;
	tf::Vector3 axis;
  ros::Rate lr(10);
  Eigen::Vector2f current_vel(0,0);
  
  Eigen::Vector2f target(goal->target_pose.pose.position.x, goal->target_pose.pose.position.y);
 /* float target_theta;
  Eigen::Quaternionf q(goal->target_pose.pose.orientation.w, goal->target_pose.pose.orientation.x, goal->target_pose.pose.orientation.y, goal->target_pose.pose.orientation.z);
  Eigen::Matrix3f m(q);
  target_theta=atan2(m(1,0),m(0,0));
  target_theta=(target_theta<0)? target_theta+2*M_PI : (target_theta>2*M_PI)? target_theta-2*M_PI : target_theta;*/
  
  while(1)
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
	   /* robot_orient=(robot_orient<0)? robot_orient+2*M_PI : (robot_orient>2*M_PI)? robot_orient-2*M_PI : robot_orient;  
	    if((robot_orient-target_theta)>M_PI) robot_orient-=2*M_PI;
	    else if((robot_orient-target_theta)<-M_PI) robot_orient+=2*M_PI;
	   */ 
	    // 19.7 28.2
	    // 2.17 4.88
	    // -.27, -.94
      // -2.27, -.4
      
      //-2.17177391052, -0.441132843494
      //-1.01823496819, -0.79
      //0.536493480206, 1.77734947205
      //2.64069199562, 0.474561214447

      Eigen::Vector2f repulsion,attraction,vel, robot_pos;

      robot_pos=Eigen::Vector2f(robot_posx,robot_posy);
      
      geometry_msgs::Twist cmd_vel;
     // if ((robot_pos-target).norm()>.05){

        computeMapImage(robot_posx, robot_posy, robot_orient);
        computeRepulsionFromImages(repulsion);
        
        attraction=target-robot_pos;
        float attr_norm=attraction.norm(); float k=attr_norm;
        attraction=(attraction/k);
        k=(k>.3)? k=.3:(k<.1)? k=.1: k;
        attraction*=k;
        float a0=attraction(0); float a1=attraction(1);
        attraction(0)=a1*sin(robot_orient)+a0*cos(robot_orient);
		    attraction(1)=(-a0*sin(robot_orient)+a1*cos(robot_orient));
		    repulsion=repulsion*(k/20);
		    if(attr_norm<=.05) attraction=Eigen::Vector2f(0,0);
        vel=attraction+repulsion;

       // std::cout<<"pos\n"<<robot_pos<<std::endl;
        std::cout<<"repulsion\n"<<repulsion<<std::endl;
       // std::cout<<"attraction\n"<<attraction<<std::endl;
       // std::cout<<"vel\n"<<vel<<std::endl;
        
        for(int i=0;i<=1;i++){
          if (vel(i) - current_vel(i) > 10.025) current_vel(i) +=0.025;
          else if(vel(i) - current_vel(i) < -10.05) current_vel(i) -=0.05;
          else current_vel(i) = vel(i);
        }
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
    /*  if((robot_pos-target).norm()<=.1)
      {
        float theta_diff=fabs(robot_orient-target_theta);
        theta_diff=(theta_diff>.4)? .4:(theta_diff<.005)? .005:theta_diff;
        cmd_vel.angular.z=((robot_orient-target_theta)>.005)? cmd_vel.angular.z-theta_diff: ((robot_orient-target_theta)<.005)? cmd_vel.angular.z+theta_diff: cmd_vel.angular.z;
      }*/
      
      if((robot_pos-target).norm()<=.05/* && (fabs(robot_orient-target_theta)<=.005)*/)
      {
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
       // cmd_vel.angular.z=0;
        current_vel=Eigen::Vector2f(0,0);
        pub_vel.publish(cmd_vel);
        as->setSucceeded();
        return;
      }
      pub_vel.publish(cmd_vel);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("holonomic_base_planner_main: %s",ex.what());
      ROS_ERROR_STREAM("TF from " << base_frame << "  to " << map_frame);
    }
    lr.sleep();
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "holonomic_base_planner_main");
  ros::NodeHandle n;
  pub_vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Subscriber sub_laser = n.subscribe("base_scan", 1, laser_cb);
  
  mtx_scan_image.lock();
	scan_image=Scalar(0);
	mtx_scan_image.unlock();
	
	cv::FileStorage file2("/home/marco/catkin_ws/distance_map.xml", cv::FileStorage::READ);
  file2["matrix"] >> distance_map;
  file2.release();
  
  map = imread("/home/marco/catkin_ws/src/sbpl_lattice_planner/worlds/work_map.ppm", CV_LOAD_IMAGE_GRAYSCALE);
  map.at<unsigned char>(map.rows-1+map_origin_y/resolution,-map_origin_x/resolution)=255;

  map_image=Scalar(0);
  
  as =new actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction>(n, "holonomic_goto", action_cb, false);
  as->start();
  
  ros::spin();
  
  return 0; 
}

