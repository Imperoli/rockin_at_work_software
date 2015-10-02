/*
 * planner_viewer.cpp
 *
 *      Author: Marco Imperoli
 */

#include "arm_planner/utils.h"
#include <GL/glut.h>
#include <geometry_msgs/Twist.h>
#include "ros/ros.h"


#define window_width  1024
#define window_height 768

using namespace arm_planner;

float angle_x = 0, angle_y = 0;
int xBegin, yBegin, moving;
float angle_su=0, angle_sin=0;
std::vector<ControlPoint> scene_control_points,real_control_points;
std::vector<Obstacle> scene_obstacles;
std::vector<Eigen::Vector3d> scene_ee_traj;
std::vector<Eigen::Vector3d> man_area;
Eigen::Vector3d target_cartesian_position;
Eigen::Matrix3d ee_orient;

void sceneCallback( const arm_planner::PlanningSceneFrame::Ptr& msg){
  rosMsg2PlanningSceneFrame(*msg, real_control_points, scene_control_points, scene_obstacles, scene_ee_traj, target_cartesian_position, man_area, ee_orient);
}

void rot_gl_cb (const geometry_msgs::Twist::ConstPtr& msg)
{  
  angle_su+=msg->linear.x;
  angle_sin+=msg->angular.z;
    
}

void idle()
{
}

void drawEEframe(){
  Eigen::Vector3d X_rot=ee_orient*Eigen::Vector3d(.1,0,0)+scene_control_points[scene_control_points.size()-1].position;
  Eigen::Vector3d Y_rot=ee_orient*Eigen::Vector3d(0,.1,0)+scene_control_points[scene_control_points.size()-1].position;
  Eigen::Vector3d Z_rot=ee_orient*Eigen::Vector3d(0,0,.1)+scene_control_points[scene_control_points.size()-1].position;
  glBegin(GL_LINES);
  glColor3f(1,0,0);
  glVertex3f(scene_control_points[scene_control_points.size()-1].position(0),scene_control_points[scene_control_points.size()-1].position(1),scene_control_points[scene_control_points.size()-1].position(2));
  glVertex3f(X_rot(0),X_rot(1),X_rot(2));
  glColor3f(0,1,0);
  glVertex3f(scene_control_points[scene_control_points.size()-1].position(0),scene_control_points[scene_control_points.size()-1].position(1),scene_control_points[scene_control_points.size()-1].position(2));
  glVertex3f(Y_rot(0),Y_rot(1),Y_rot(2));
  glColor3f(0,0,1);
  glVertex3f(scene_control_points[scene_control_points.size()-1].position(0),scene_control_points[scene_control_points.size()-1].position(1),scene_control_points[scene_control_points.size()-1].position(2));
  glVertex3f(Z_rot(0),Z_rot(1),Z_rot(2));
  glEnd();
}

void drawManipulationArea(){
  glPointSize(2.0);
  glBegin(GL_POINTS);
  for(int i=0;i<man_area.size();i++){
    glPushMatrix();
    glColor3f(1,.8,0);
    glVertex3d(man_area[i](0),man_area[i](1),man_area[i](2));
    glPopMatrix();
  }
  glEnd();
  glPointSize(1.0);
  glColor3f(1,1,1);
}

void drawGroundPlane(){

  glPushMatrix();
  //glScalef(.3,.3,1);
  glColor3f(.4,.4,.4);
  glBegin(GL_QUADS);
  glVertex3f(-1,-1,-.1);
  glVertex3f(-1,1,-.1);
  glVertex3f(1,1,-.1);
  glVertex3f(1,-1,-.1);
  glEnd();
  glColor3f(1,1,1);
  glPopMatrix();

}

void drawTarget(){
  glBegin(GL_LINES);
  glVertex3d(target_cartesian_position(0)-.01,target_cartesian_position(1),target_cartesian_position(2));
  glVertex3d(target_cartesian_position(0)+.01,target_cartesian_position(1),target_cartesian_position(2));

  glVertex3d(target_cartesian_position(0),target_cartesian_position(1)-.01,target_cartesian_position(2));
  glVertex3d(target_cartesian_position(0),target_cartesian_position(1)+.01,target_cartesian_position(2));
  
  glVertex3d(target_cartesian_position(0),target_cartesian_position(1),target_cartesian_position(2)-.01);
  glVertex3d(target_cartesian_position(0),target_cartesian_position(1),target_cartesian_position(2)+.01);
  glEnd();
}

void drawEETraj(){
  glBegin(GL_POINTS);
  for(int i=0;i<scene_ee_traj.size();i++){
    glPushMatrix();
    glColor3f(1,1,1);
    glVertex3d(scene_ee_traj[i](0),scene_ee_traj[i](1),scene_ee_traj[i](2));
    glPopMatrix();
  }
  glEnd();
}

void drawControlPoints(){
  for(int i=0;i<scene_control_points.size();i++){
    glPushMatrix();
    if(scene_control_points[i].link==0)
      glColor4f(0,0,1,.6);
    else if(scene_control_points[i].link==2)
      glColor4f(0,1,0,.6);
    else if(scene_control_points[i].link==3)
      glColor4f(1,0,0,.6);
    else
      glColor4f(1,1,1,.6);
    glTranslated(scene_control_points[i].position(0),scene_control_points[i].position(1),scene_control_points[i].position(2));
    glutSolidSphere(scene_control_points[i].radius,20,20);
    glPopMatrix();
  }
  for(int i=0;i<real_control_points.size();i++){
    glPushMatrix();
    if(real_control_points[i].link==0)
      glColor3f(0,0,1);
    else if(real_control_points[i].link==2)
      glColor3f(0,1,0);
    else if(real_control_points[i].link==3)
      glColor3f(1,0,0);
    else
      glColor3f(1,1,1);
    glTranslated(real_control_points[i].position(0),real_control_points[i].position(1),real_control_points[i].position(2));
    glutSolidSphere(real_control_points[i].radius,20,20);
    glPopMatrix();
  }
  glColor3f(1,1,1);
}

void drawObstacles(){
  for(int i=0;i<scene_obstacles.size();i++){
    glPushMatrix();
      glColor3f(1,0,1);
    if(scene_obstacles[i].radius>=0){
      glTranslated(scene_obstacles[i].position(0),scene_obstacles[i].position(1),scene_obstacles[i].position(2));
      glutSolidSphere(scene_obstacles[i].radius,20,20);
    }else{
      glBegin(GL_QUADS);
      glNormal3f(0,0,-1);
      glVertex3d(scene_obstacles[i].min_position(0),scene_obstacles[i].min_position(1),scene_obstacles[i].min_position(2));
      glVertex3d(scene_obstacles[i].min_position(0),scene_obstacles[i].max_position(1),scene_obstacles[i].min_position(2));
      glVertex3d(scene_obstacles[i].max_position(0),scene_obstacles[i].max_position(1),scene_obstacles[i].min_position(2));
      glVertex3d(scene_obstacles[i].max_position(0),scene_obstacles[i].min_position(1),scene_obstacles[i].min_position(2));
  
      glNormal3f(-1,0,0);
      glVertex3d(scene_obstacles[i].min_position(0),scene_obstacles[i].min_position(1),scene_obstacles[i].min_position(2));
      glVertex3d(scene_obstacles[i].min_position(0),scene_obstacles[i].max_position(1),scene_obstacles[i].min_position(2));
      glVertex3d(scene_obstacles[i].min_position(0),scene_obstacles[i].max_position(1),scene_obstacles[i].max_position(2));
      glVertex3d(scene_obstacles[i].min_position(0),scene_obstacles[i].min_position(1),scene_obstacles[i].max_position(2));

      glNormal3f(0,0,1);
      glVertex3d(scene_obstacles[i].min_position(0),scene_obstacles[i].min_position(1),scene_obstacles[i].max_position(2));
      glVertex3d(scene_obstacles[i].min_position(0),scene_obstacles[i].max_position(1),scene_obstacles[i].max_position(2));
      glVertex3d(scene_obstacles[i].max_position(0),scene_obstacles[i].max_position(1),scene_obstacles[i].max_position(2));
      glVertex3d(scene_obstacles[i].max_position(0),scene_obstacles[i].min_position(1),scene_obstacles[i].max_position(2));

      glNormal3f(1,0,0);
      glVertex3d(scene_obstacles[i].max_position(0),scene_obstacles[i].min_position(1),scene_obstacles[i].min_position(2));
      glVertex3d(scene_obstacles[i].max_position(0),scene_obstacles[i].max_position(1),scene_obstacles[i].min_position(2));
      glVertex3d(scene_obstacles[i].max_position(0),scene_obstacles[i].max_position(1),scene_obstacles[i].max_position(2));
      glVertex3d(scene_obstacles[i].max_position(0),scene_obstacles[i].min_position(1),scene_obstacles[i].max_position(2));

      glNormal3f(0,1,0);
      glVertex3d(scene_obstacles[i].min_position(0),scene_obstacles[i].max_position(1),scene_obstacles[i].min_position(2));
      glVertex3d(scene_obstacles[i].min_position(0),scene_obstacles[i].max_position(1),scene_obstacles[i].max_position(2));
      glVertex3d(scene_obstacles[i].max_position(0),scene_obstacles[i].max_position(1),scene_obstacles[i].max_position(2));
      glVertex3d(scene_obstacles[i].max_position(0),scene_obstacles[i].max_position(1),scene_obstacles[i].min_position(2));

      glNormal3f(0,-1,0);
      glVertex3d(scene_obstacles[i].min_position(0),scene_obstacles[i].min_position(1),scene_obstacles[i].min_position(2));
      glVertex3d(scene_obstacles[i].min_position(0),scene_obstacles[i].min_position(1),scene_obstacles[i].max_position(2));
      glVertex3d(scene_obstacles[i].max_position(0),scene_obstacles[i].min_position(1),scene_obstacles[i].max_position(2));
      glVertex3d(scene_obstacles[i].max_position(0),scene_obstacles[i].min_position(1),scene_obstacles[i].min_position(2));
      glEnd();
    }
    glPopMatrix();
  }
}

// glut Main loop
void glut_loop_function()
{

   // Clear color (screen) 
   // And depth (used internally to block obstructed objects)
   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
   // Load identity matrix
   glLoadIdentity();
  
  gluLookAt(0,2,2,
    0,0,0,
    0,0,1);

  glRotatef(angle_su, 1, 0, 0);
  glRotatef(angle_sin, 0, 0, 1);
  
  glScalef(1,1,1);
  drawEEframe();  
  drawGroundPlane();
  drawManipulationArea();
  drawTarget();
  drawEETraj();
  drawObstacles();
  drawControlPoints();

  glutSwapBuffers();
  //glutPostRedisplay();
}

void GL_Setup(int width, int height)
{
  glViewport( 0, 0, width, height );
  glMatrixMode( GL_PROJECTION );
  glEnable(GL_DEPTH_TEST);
  glDepthMask(GL_TRUE);
  glDepthFunc(GL_LEQUAL);
  glDepthRange(0.0f, 1.0f);
  gluPerspective( 45, (float)width/height, .1, 10 );
  glMatrixMode( GL_MODELVIEW );
  glEnable(GL_COLOR_MATERIAL);
  glEnable(GL_NORMALIZE);
  
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); 
  glEnable( GL_BLEND );

  glEnable(GL_LIGHTING);
  glShadeModel(GL_SMOOTH);
  glEnable(GL_LIGHT0);

  glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

  // Create light components
  float ambientLight[] = { 0.2f, 0.2f, 0.2f, 1.0f };
  float diffuseLight[] = { 0.8f, 0.8f, 0.8, 1.0f };
  float specularLight[] = { 0.5f, 0.5f, 0.5f, 1.0f };
  float position[] = { 0.0f, 100.0f, 100.0f, 1.0f };

  // Assign created components to GL_LIGHT0
  glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);
  glLightfv(GL_LIGHT0, GL_SPECULAR, specularLight);
  glLightfv(GL_LIGHT0, GL_POSITION, position);
}

int main(int argc, char **argv){

  glutInit(&argc, argv);
  glutInitWindowSize(window_width, window_height);
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
  glutCreateWindow("youbout_arm");
  glutIdleFunc(idle);
  GL_Setup(window_width, window_height);

  ros::init(argc, argv, "planner_viewer");
    ros::NodeHandle nh;
  //ros::AsyncSpinner spinner(8);

  ros::Subscriber cmd_sub = nh.subscribe("rot_gl", 1, rot_gl_cb);
  ros::Subscriber scene_sub = nh.subscribe("/planning_scene_frame", 1, sceneCallback);
  ros::spinOnce();
  //spinner.start();
  sleep(.1);
  ros::spinOnce();
  ros::Rate lr(24);
  while(nh.ok()){
    ros::spinOnce();
    if(scene_control_points.size()>0&&scene_obstacles.size()>0&&scene_ee_traj.size()>0){
      glut_loop_function();
    }
    lr.sleep();
  }

  return 1;

}


