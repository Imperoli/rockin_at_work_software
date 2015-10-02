cmake_minimum_required(VERSION 2.8.3)
project(youbot_calibration)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  #moveit_ros_planning_interface
  roscpp
  roslib
  rospy
  std_msgs
  std_srvs
  tf
  tf_conversions
)

find_package(Eigen3 REQUIRED)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rockin
#  CAT#KIN_DEPENDS actionlib geometry_msgs move_base_msgs moveit_ros_planning_interface roscpp roslib rospy std_msgs std_srvs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  /usr/include/eigen3/
)
## Declare a cpp library
# add_library(rockin
#   src/${PROJECT_NAME}/rockin.cpp
# )

## Declare a cpp executable

  add_executable(kinect_arm_calib src/kinect_arm_calib.cpp)
  add_executable(camera_arm_calib src/camera_arm_calib.cpp)
 
#add_dependencies(planner_main ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_arm_calib ${catkin_LIBRARIES})
target_link_libraries(kinect_arm_calib ${catkin_LIBRARIES})




