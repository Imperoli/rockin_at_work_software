#include <iostream>
#include "opencv2/core/core.hpp"
#include <math.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "Eigen/Dense"

float resolution=0.05;
float radius=.8;
float base_x=0.65;
float base_y=.4;

float map_origin_x=-7.15/resolution; float map_origin_y=-3.8/resolution;

using namespace cv;

int scale=3;

static void onMouse( int event, int x, int y, int, void* )
{
 // if( event != EVENT_LBUTTONDOWN )
 // return;
  x/=scale;
  y/=scale;
  float px=(float)(x)*resolution+map_origin_x*resolution;
  float py=(float)(200-y)*resolution+map_origin_y*resolution;
  std::cout<<"x "<<px<<" y "<<py<<std::endl;
  
  if( event == EVENT_LBUTTONDOWN ) std::cout<<"x "<<px<<" y "<<py<<" GO"<<std::endl;
}

int main(int argc, char **argv)
{
  std::vector<Eigen::Vector2f> points;
  int count=0;
  int inner_square=(int)round(.0/resolution);
  int outer_square=(int)round(1.15/resolution);
  
  Mat map = imread("/home/marco/catkin_ws/src/sbpl_lattice_planner/worlds/work_map.ppm", CV_LOAD_IMAGE_GRAYSCALE);
  for(int i=round(outer_square/2);i<round(map.rows-outer_square/2);i++){
    for(int j=round(outer_square/2);j<round(map.cols-outer_square/2);j++){
     // std::cout<<"iter "<<(int)map.at<unsigned char>(i,j)<<std::endl;
      if(map.at<unsigned char>(i,j)>210)
      {
        bool ok=true;
        for(int h=i-round(outer_square/2); h<i+round(outer_square/2); h++){
          for(int k=j-round(outer_square/2); k<j+round(outer_square/2); k++){
            if(map.at<unsigned char>(h,k)<=210)
            {
              ok=false;
              break;
            }
          }
        }
        if(ok)
        {
          for(int h=i-round(inner_square/2); h<i+round(inner_square/2); h++){
            for(int k=j-round(inner_square/2); k<j+round(inner_square/2); k++){
              map.at<unsigned char>(h,k)=190;
            }
          }
          map.at<unsigned char>(i,j)=50;
          Eigen::Vector2f p((float)j*resolution+map_origin_x*resolution, (float)(map.rows-i)*resolution+map_origin_y*resolution);
          points.push_back(p);
          count++;
        }
      }
    }
  }
  
  std::cout<<"COUNT "<<count<<std::endl;
  
/*  std::vector<Vec3b> colors;
  RNG rng( 0xFFFFFFFF );
  for(size_t k=0;k<points.size();k++){
     int icolor = (unsigned) rng;
     colors.push_back(Vec3b(icolor&255, (icolor>>8)&255, (icolor>>16)&255 ));
  }
  
  Mat out; out=Mat::zeros(map_origin.rows, map_origin.cols, CV_8UC3);
  for(int i=0; i<map_origin.rows; i++){
    std::cout<<"iter "<<i<<std::endl;
    for(int j=0; j<map_origin.cols; j++){
      if(map_origin.at<unsigned char>(i,j)>110)
      { 
        float min_dist=1000;
        for(size_t k=0;k<points.size();k++){
          Eigen::Vector2f p((float)j*resolution, (float)i*resolution);
          if((p-points[k]).norm()<min_dist)
          {
            min_dist=(p-points[k]).norm();
            out.at<Vec3b>(i,j)=colors[k];
          }
        }
      }
    }
  }
  */
  resize(map, map, cv::Size(map.cols*scale,map.rows*scale), 0, 0,cv::INTER_NEAREST);
  while(1)
  {
    cv::imshow("GUI",map);
    setMouseCallback( "GUI", onMouse, 0 );
	  cv::waitKey(0);
	}
	return 0;
}



