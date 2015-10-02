#include <ros/ros.h>
#include <math.h>
#include <sensor_msgs/LaserScan.h>

ros::Publisher pub;

void callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  sensor_msgs::LaserScan out;
  out=*msg;
	float range_scan_min=msg->range_min;
	float range_scan_max=msg->range_max;

  int range=200; int offset=100;
  for(size_t i = 0; i < msg->ranges.size(); i++){
		if(msg->ranges[i]>range_scan_min&&msg->ranges[i]<range_scan_max)
		{
      float noise=(float)(rand()%range-offset)/10000.0;
      out.ranges[i]+=msg->ranges[i]*noise*1;
    }
	}
  pub.publish(out);
}


int main(int argc, char **argv)
{

	ros::init(argc, argv, "base_sca_noisy");


	ros::NodeHandle n;

	pub = n.advertise<sensor_msgs::LaserScan>("/base_scan_noisy", 1);
	ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/base_scan", 1, callback);
	//ros::spin();

  ros::spin();

	return 0;
}

