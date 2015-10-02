#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <iostream>
#include <fstream>
#include <string>
#include "opencv2/core/core.hpp"
#include <math.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;

struct mapInfo
{
  std::string file_name;
  float resolution;
  float map_origin_x;
  float map_origin_y;
  float radius;
  int im_size;
  int map_treshold;
  int map_scale;
};

struct robotInfo
{
  float base_x;
  float base_y;
  float laser_x;
};

void readMapYamlFile(char* dirname, mapInfo& info)
{
  std::string dir(dirname);
  
  FILE* file=fopen((dir+"/map.yaml").c_str(),"rt");
  char value[255];
  while(fscanf(file,"%s",value)!=EOF)
  {
    if( strcmp (value, "image:") == 0) 
    {
      fscanf(file,"%s",value);
      info.file_name=dir+"/"+value;
    }
    if( strcmp (value, "resolution:") == 0) 
    {
      fscanf(file,"%s",value);
      info.resolution=atof(value);
    }
    if( strcmp (value, "origin_x:") == 0) 
    {
      fscanf(file,"%s",value);
      info.map_origin_x=atof(value);
    }
    if( strcmp (value, "origin_y:") == 0) 
    {
      fscanf(file,"%s",value);
      info.map_origin_y=atof(value);
    }
    if( strcmp (value, "radius:") == 0) 
    {
      fscanf(file,"%s",value);
      info.radius=atof(value);
    }
    if( strcmp (value, "map_scale:") == 0) 
    {
      fscanf(file,"%s",value);
      info.map_scale=atoi(value);
    }
  }
  fclose(file);
  info.im_size=round(info.radius*2/info.resolution);
  info.map_treshold=135;
}

void readRobotYamlFile(char* dirname, robotInfo& info)
{
  std::string dir(dirname);
  
  FILE* file=fopen((dir+"/robot.yaml").c_str(),"rt");
  char value[255];
  while(fscanf(file,"%s",value)!=EOF)
  {
    if( strcmp (value, "base_x:") == 0) 
    {
      fscanf(file,"%s",value);
      info.base_x=atof(value);
    }
    if( strcmp (value, "base_y:") == 0) 
    {
      fscanf(file,"%s",value);
      info.base_y=atof(value);
    }
    if( strcmp (value, "laser_x:") == 0) 
    {
      fscanf(file,"%s",value);
      info.laser_x=atof(value);
    }
  }
  fclose(file);
}

void readInfoFiles(char* argv, mapInfo& map_info, robotInfo& robot_info)
{
  readMapYamlFile(argv, map_info);
  readRobotYamlFile(argv, robot_info);
}

void compute_checkpoints(mapInfo& map_info, std::vector<Eigen::Vector2f>& points)
{
  float resolution=map_info.resolution;
  float map_origin_x=map_info.map_origin_x;
  float map_origin_y=map_info.map_origin_y;
  points.clear();
  int count=0;
  int inner_square=(int)round(.3/resolution);
  int outer_square=(int)round(1.15/resolution);
  
  Mat map = imread(map_info.file_name, CV_LOAD_IMAGE_GRAYSCALE);
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
              map.at<unsigned char>(h,k)=211;//190;
            }
          }
          map.at<unsigned char>(i,j)=50;
          Eigen::Vector2f p((float)j*resolution+map_origin_x, (float)(map.rows-i)*resolution+map_origin_y);
          points.push_back(p);
          count++;
        }
      }
    }
  } 
  std::cout<<"chekpoints COUNT "<<count<<std::endl;
  
  for(int i=0;i<map.rows;i++){
    for(int j=0;j<map.cols;j++){
      if(map.at<unsigned char>(i,j)==211||map.at<unsigned char>(i,j)==50)
        map.at<unsigned char>(i,j)=65;
    }
  }
  
  if (map_info.map_scale>1)
  {
    resize(map, map, cv::Size(map.cols*map_info.map_scale,map.rows*map_info.map_scale), 0, 0,cv::INTER_NEAREST);
  }
  cv::imshow("checkpoints",map);
  cv::waitKey(0);

}

void compute_distance_map(mapInfo& map_info, robotInfo& robot_info, Mat& out)
{
  float radius=map_info.radius;
  float resolution=map_info.resolution;
  float base_x=robot_info.base_x;
  float base_y=robot_info.base_y;
  int im_size=round(radius*2/resolution);
  out=Mat(im_size,im_size,CV_32FC3);
  out=cv::Scalar(0,0,255);
  Vec2i A(round((im_size/2)-round((base_y/2)/resolution)), round((im_size/2)-round((base_x/2)/resolution)));
  Vec2i B(round((im_size/2)-round((base_y/2)/resolution)), round((im_size/2)+round((base_x/2)/resolution)));
  Vec2i C(round((im_size/2)+round((base_y/2)/resolution)), round((im_size/2)+round((base_x/2)/resolution)));
  Vec2i D(round((im_size/2)+round((base_y/2)/resolution)), round((im_size/2)-round((base_x/2)/resolution)));
  for(int i=0; i<im_size; i++){
    for(int j=0; j<im_size; j++){
      
      Vec2i v(i,j);
      Vec2i A2=A;
      while(A2!=B)
      {
        float dist=norm(v-A2)*resolution;
        //float px=A2[1]; float py=A2[0]; float dist_px=norm(v-A2);
        Vec2f dir((v[0]-A2[0])*resolution, (v[1]-A2[1])*resolution);
        float px=dir[1]; float py=dir[0];
        if(dist<out.at<Vec3f>(i,j)[2])
        {
          Vec3f value(px,-py,dist);
          if (dist!=0)
          {
            out.at<Vec3f>(i,j)=value/dist;
            out.at<Vec3f>(i,j)[2]=dist;
          }
          else 
          {
            if(A2==A) out.at<Vec3f>(i,j)=Vec3f(-1,1,.02);
            else out.at<Vec3f>(i,j)=Vec3f(0,1,.02);
          }
        }
        A2[1]++;
      }
      Vec2i B2=B;
      while(B2!=C)
      {
        float dist=norm(v-B2)*resolution;
        //float px=A2[1]; float py=A2[0]; float dist_px=norm(v-A2);
        Vec2f dir((v[0]-B2[0])*resolution, (v[1]-B2[1])*resolution);
        float px=dir[1]; float py=dir[0];
        if(dist<out.at<Vec3f>(i,j)[2])
        {
          Vec3f value(px,-py,dist);
           if (dist!=0)
           {
            out.at<Vec3f>(i,j)=value/dist;
             out.at<Vec3f>(i,j)[2]=dist;
           }
         else 
          {
            if(B2==B) out.at<Vec3f>(i,j)=Vec3f(1,1,.02);
            else out.at<Vec3f>(i,j)=Vec3f(1,0,.02);
          }
        }
        B2[0]+=1;
      }
      Vec2i C2=C;
      while(C2!=D)
      {
        float dist=norm(v-C2)*resolution;
       //float px=A2[1]; float py=A2[0]; float dist_px=norm(v-A2);
        Vec2f dir((v[0]-C2[0])*resolution, (v[1]-C2[1])*resolution);
        float px=dir[1]; float py=dir[0]; 
        if(dist<out.at<Vec3f>(i,j)[2])
        {
          Vec3f value(px,-py,dist);
          if (dist!=0)
          {
            out.at<Vec3f>(i,j)=value/dist;
            out.at<Vec3f>(i,j)[2]=dist;
          }
          else 
          {
           if(C2==C) out.at<Vec3f>(i,j)=Vec3f(1,-1,.02);
            else out.at<Vec3f>(i,j)=Vec3f(0,-1,.02);
          }
        }
        C2[1]-=1;
      }
      Vec2i D2=D;
      while(D2!=A)
      {
        float dist=norm(v-D2)*resolution;
       //float px=A2[1]; float py=A2[0]; float dist_px=norm(v-A2);
        Vec2f dir((v[0]-D2[0])*resolution, (v[1]-D2[1])*resolution);
        float px=dir[1]; float py=dir[0]; 
        if(dist<out.at<Vec3f>(i,j)[2])
        {
          Vec3f value(px,-py,dist);
           if (dist!=0)
           {
            out.at<Vec3f>(i,j)=value/dist;
             out.at<Vec3f>(i,j)[2]=dist;
           }
          else 
          {
            if(D2==D) out.at<Vec3f>(i,j)=Vec3f(-1,-1,.02);
            else out.at<Vec3f>(i,j)=Vec3f(-1,0,.02);
          }
        }
        D2[0]-=1;
      }
    }
  }
}



#endif


