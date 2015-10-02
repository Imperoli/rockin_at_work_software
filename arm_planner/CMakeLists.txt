cmake_minimum_required(VERSION 2.8.3)
project(arm_planner)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
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
  rpg_youbot_common
  trajectory_generator
)

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
 add_message_files(
   FILES
   Obstacle_msg.msg
   ControlPoint_msg.msg
   PlanningSceneFrame.msg
 #  PlanningRequest.msg
   TargetRequest.msg
   Obstacles.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   check_IK_feasibility.srv
   get_ee_pose.srv
#   ArmMotionPlanning.srv
#   Service2.srv
 )

## Generate actions in the 'action' folder
 add_action_files(
   FILES
   arm_planning.action
#   Action2.action
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs
   actionlib_msgs
   #   move_base_msgs#   std_msgs
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
#  LIBRARIES rockin
#  CATKIN_DEPENDS actionlib geometry_msgs move_base_msgs moveit_ros_planning_interface roscpp roslib rospy std_msgs std_srvs tf
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
  add_executable(arm_frame_publisher src/arm_frame_publisher.cpp)
 add_executable(teleop_omni_key src/teleop_omni_key.cpp)

 add_executable(planner_main src/planner_main.cpp src/planner.cpp)
 add_executable(planner_viewer src/planner_viewer.cpp )
 add_executable(planning_request_node src/planning_request_node.cpp )
 add_executable(planning_request_node_clicked_point src/planning_request_node_clicked_point.cpp )

 add_executable(key_for_opengl src/key_for_opengl.cpp)

 add_executable(compute_workspace src/compute_workspace.cpp src/planner.cpp)
 
 add_executable(compute_grid src/compute_grid.cpp )

add_dependencies(compute_grid ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(arm_frame_publisher ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(planner_main ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(planner_viewer ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(planning_request_node ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(planning_request_node_clicked_point ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(compute_workspace ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(compute_grid ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(compute_grid ${catkin_LIBRARIES})
target_link_libraries(arm_frame_publisher ${catkin_LIBRARIES})
target_link_libraries(teleop_omni_key ${catkin_LIBRARIES})
target_link_libraries(planner_main ${catkin_LIBRARIES} )
target_link_libraries(planner_viewer ${catkin_LIBRARIES} GL glut GLU)
target_link_libraries(key_for_opengl ${catkin_LIBRARIES})
target_link_libraries(planning_request_node ${catkin_LIBRARIES} )
target_link_libraries(planning_request_node_clicked_point ${catkin_LIBRARIES} )
target_link_libraries(compute_workspace ${catkin_LIBRARIES} )

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(rockin_node rockin_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(rockin_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS rockin rockin_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rockin.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
