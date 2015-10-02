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

int main(int argc, char **argv)
{
  
  int im_size=round(radius*2/resolution);
  Mat out(im_size,im_size,CV_32FC3);
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
        float norm=Eigen::Vector2f(out.at<Vec3f>(i,j)[0],out.at<Vec3f>(i,j)[1]).norm();
        if((1/dist)>norm)
        {
          Vec3f value(px,-py,dist);
          if (dist!=0)
          {
            out.at<Vec3f>(i,j)=value/(dist*dist);
            out.at<Vec3f>(i,j)[2]=0;
          }
          else 
          {
            if(A2==A) out.at<Vec3f>(i,j)=Vec3f(-0.707/.02,0.707/.02,0);
            else out.at<Vec3f>(i,j)=Vec3f(0,1/.02,0);
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
        float norm=Eigen::Vector2f(out.at<Vec3f>(i,j)[0],out.at<Vec3f>(i,j)[1]).norm();
        if((1/dist)>norm)
        {
          Vec3f value(px,-py,dist);
           if (dist!=0)
           {
            out.at<Vec3f>(i,j)=value/(dist*dist);
            out.at<Vec3f>(i,j)[2]=1;
           }
         else 
          {
            if(B2==B) out.at<Vec3f>(i,j)=Vec3f(0.707/.02,0.707/.02,1);
            else out.at<Vec3f>(i,j)=Vec3f(1/.02,0,1);
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
        float norm=Eigen::Vector2f(out.at<Vec3f>(i,j)[0],out.at<Vec3f>(i,j)[1]).norm();
        if((1/dist)>norm)
        {
          Vec3f value(px,-py,dist);
          if (dist!=0)
          {
            out.at<Vec3f>(i,j)=value/(dist*dist);
            out.at<Vec3f>(i,j)[2]=2;
          }
          else 
          {
           if(C2==C) out.at<Vec3f>(i,j)=Vec3f(0.707/.02,-0.707/.02,2);
            else out.at<Vec3f>(i,j)=Vec3f(0,-1/.02,2);
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
        float norm=Eigen::Vector2f(out.at<Vec3f>(i,j)[0],out.at<Vec3f>(i,j)[1]).norm();
        if((1/dist)>norm)
        {
          Vec3f value(px,-py,dist);
           if (dist!=0)
           {
            out.at<Vec3f>(i,j)=value/(dist*dist);
            out.at<Vec3f>(i,j)[2]=3;
           }
          else 
          {
            if(D2==D) out.at<Vec3f>(i,j)=Vec3f(-0.707/.02,-0.707/.02,3);
            else out.at<Vec3f>(i,j)=Vec3f(-1/.02,0,3);
          }
        }
        D2[0]-=1;
      }
    }
  }
  
  cv::FileStorage file("distance_map.xml", cv::FileStorage::WRITE);
  // Write to file!
  file << "matrix"<< out;
  file.release();
  
  cv::FileStorage file2("distance_map.xml", cv::FileStorage::READ);
  Mat out2;
  file2["matrix"] >> out2;
  file2.release();
  
  
  for(int i=0;i<out2.rows;i++){
    for(int j=0; j<out2.cols;j++){
      //if (out.at<Vec3f>(i,j)[2]!=0)
        //out.at<Vec3f>(i,j)=Vec3f(((out.at<Vec3f>(i,j)[0]/out.at<Vec3f>(i,j)[2])+1)/2, ((out.at<Vec3f>(i,j)[1]/out.at<Vec3f>(i,j)[2])+1)/2, .6);
     // else
        out2.at<Vec3f>(i,j)=Vec3f((out2.at<Vec3f>(i,j)[0]+1)/2, (out2.at<Vec3f>(i,j)[1]+1)/2, 1);
        out.at<Vec3f>(i,j)=Vec3f((out.at<Vec3f>(i,j)[0]+1)/2, (out.at<Vec3f>(i,j)[1]+1)/2, 1);
    }
  }

  Mat src_gray = imread("/home/marco/catkin_ws/src/sbpl_lattice_planner/worlds/work_map.ppm", CV_LOAD_IMAGE_GRAYSCALE);
 /* 
  //Mat src_gray;
  Mat grad;
  int scale = 1;
  int delta = 0;
  int ddepth = CV_16S;
  //cvtColor( src, src_gray, CV_RGB2GRAY );

  /// Generate grad_x and grad_y
  Mat grad_x, grad_y;
  Mat abs_grad_x, abs_grad_y;

  /// Gradient X
  //Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
  Sobel( src_gray, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
  convertScaleAbs( grad_x, abs_grad_x );

  /// Gradient Y
  //Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
  Sobel( src_gray, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
  convertScaleAbs( grad_y, abs_grad_y );

  /// Total Gradient (approximate)
  addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );
  imshow( "ciao", grad );
  cv::waitKey(0);*/
  for(int i=0;i<src_gray.rows;i++){
    for(int j=0;j<src_gray.cols;j++){
      if(i==0 || i==src_gray.rows-1 || j==0 || j==src_gray.cols-1) src_gray.at<unsigned char>(i,j)=0;
    }
  }
  imshow( "ciao", src_gray );
  cv::waitKey(0);
  imwrite("/home/marco/catkin_ws/src/sbpl_lattice_planner/worlds/work_map2.ppm", src_gray);
  
  resize(out2, out2, cv::Size(800,800), 0, 0,cv::INTER_NEAREST);
  cv::imshow("GUI",out2);
	cv::waitKey(0);
	return 0;
}



