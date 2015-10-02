cmake_minimum_required(VERSION 2.8.3)
project(youbot_manipulation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  tf
  tf_conversions
  eigen_conversions
  actionlib
  mcr_perception_msgs
  arm_planner
  youbot_object_recognition
)


find_package(Boost REQUIRED)  
find_package(Eigen3 REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
#add_service_files(
#  FILES
#  DetectObjects.srv
#)

## Generate actions in the 'action' folder
 add_action_files(
   FILES
   youbot_manipulation.action
#   Action2.action
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  mcr_perception_msgs
  actionlib_msgs
)

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
  INCLUDE_DIRS include
#  LIBRARIES rockin_obstacle_detection
#  CATKIN_DEPENDS arm_planner pcl_ros roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

include_directories( 
                      include/
                      ${catkin_INCLUDE_DIRS}
                      ${Boost_INCLUDE_DIRS}
                      ${EIGEN3_INCLUDE_DIR}
)

## Declare a cpp library
# add_library(rockin_obstacle_detection
#   src/${PROJECT_NAME}/rockin_obstacle_detection.cpp
# )

## Declare a cpp executable
#add_executable(youbot_manipulation_server src/youbot_manipulation_server.cpp)
#add_executable(youbot_manipulation_client src/youbot_manipulation_client.cpp)
add_executable(youbot_manipulation_server_next_best_view_experiments src/youbot_manipulation_server_next_best_view_experiments.cpp)
add_executable(youbot_manipulation_server_next_best_view src/youbot_manipulation_server_next_best_view.cpp)
add_executable(youbot_manipulation_server_multiviews src/youbot_manipulation_server_multiviews.cpp)
add_executable(youbot_manipulation_server_rockin src/youbot_manipulation_server_rockin.cpp)
add_executable(youbot_manipulation_server_demo src/youbot_manipulation_server_demo.cpp)
add_executable(youbot_manipulation_client_demo src/youbot_manipulation_client_demo.cpp)
add_executable(youbot_manipulation_client_rockin src/youbot_manipulation_client_rockin.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(youbot_manipulation_server_next_best_view_experiments youbot_manipulation_gencpp youbot_object_recognition_gencpp mcr_object_detection_gencpp)
add_dependencies(youbot_manipulation_server_next_best_view youbot_manipulation_gencpp youbot_object_recognition_gencpp mcr_object_detection_gencpp)
add_dependencies(youbot_manipulation_server_multiviews youbot_manipulation_gencpp youbot_object_recognition_gencpp mcr_object_detection_gencpp)
add_dependencies(youbot_manipulation_server_rockin youbot_manipulation_gencpp youbot_object_recognition_gencpp mcr_object_detection_gencpp)
add_dependencies(youbot_manipulation_server_demo youbot_manipulation_gencpp youbot_object_recognition_gencpp mcr_object_detection_gencpp)
add_dependencies(youbot_manipulation_client_demo youbot_manipulation_gencpp youbot_object_recognition_gencpp mcr_object_detection_gencpp)
add_dependencies(youbot_manipulation_client_rockin youbot_manipulation_gencpp youbot_object_recognition_gencpp mcr_object_detection_gencpp)
#add_dependencies(youbot_manipulation_server youbot_manipulation_gencpp youbot_object_recognition_gencpp mcr_object_detection_gencpp)
#add_dependencies(youbot_manipulation_client youbot_manipulation_gencpp youbot_object_recognition_gencpp mcr_object_detection_gencpp)



## Specify libraries to link a library or executable target against
target_link_libraries( youbot_manipulation_server_next_best_view_experiments
                        ${catkin_LIBRARIES})
                        
target_link_libraries( youbot_manipulation_server_next_best_view
                        ${catkin_LIBRARIES})

target_link_libraries( youbot_manipulation_server_multiviews
                        ${catkin_LIBRARIES})

target_link_libraries( youbot_manipulation_server_rockin
                        ${catkin_LIBRARIES})

target_link_libraries( youbot_manipulation_server_demo
                        ${catkin_LIBRARIES})
                        
target_link_libraries( youbot_manipulation_client_demo
                        ${catkin_LIBRARIES})
                        
target_link_libraries( youbot_manipulation_client_rockin
                        ${catkin_LIBRARIES})



