cmake_minimum_required(VERSION 2.8.3)
project(holonomic_base_planner)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  move_base_msgs
  #moveit_ros_planning_interface
  roscpp
  roslib
  rospy
  std_msgs
  std_srvs
  tf
  tf_conversions
  cv_bridge
  image_transport
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL 1.6 REQUIRED)


## Generate messages in the 'msg' folder
 #add_message_files(
 #  FILES
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   ArmMotionPlanning.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
 #add_action_files(
 #  FILES
 #  arm_planning.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 #generate_messages(
 #  DEPENDENCIES
 #  geometry_msgs
 #  actionlib_msgs
   #   move_base_msgs#   std_msgs
 #)

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
  ${PCL_INCLUDE_DIRS}
  /usr/include/eigen3/
  
)
## Declare a cpp library
# add_library(rockin
#   src/${PROJECT_NAME}/rockin.cpp
# )

## Declare a cpp executable

  add_executable(multiple_scan_merger src/multiple_scan_merger.cpp)
  add_executable(kinect_scan src/kinect_scan.cpp)
  add_executable(base_scan_noisy src/base_scan_noisy.cpp)
# add_executable(holonomic_base_planner_main src/holonomic_base_planner_main.cpp)
 add_executable(holonomic_base_planner_ActionServer src/holonomic_base_planner_ActionServer.cpp)
 add_executable(holonomic_base_planner_ActionClient src/holonomic_base_planner_ActionClient.cpp)
  add_executable(base_planner_ActionServer src/base_planner_ActionServer.cpp)

#add_dependencies(planner_main ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(multiple_scan_merger ${catkin_LIBRARIES})
target_link_libraries(kinect_scan ${catkin_LIBRARIES}  ${PCL_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(base_scan_noisy ${catkin_LIBRARIES})
target_link_libraries(base_planner_ActionServer ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(holonomic_base_planner_ActionClient ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(holonomic_base_planner_ActionServer ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBS})
#target_link_libraries(holonomic_base_planner_main ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBS})



