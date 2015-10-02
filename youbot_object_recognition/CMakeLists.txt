cmake_minimum_required(VERSION 2.8.3)
project(youbot_object_recognition)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  message_generation
  tf
  tf_conversions
  actionlib
  mcr_object_detection
  eigen_conversions
)

find_package(OpenMP REQUIRED)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${OpenMP_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${OpenMP_CXX_FLAGS}")

# 
find_package( OpenCV REQUIRED )
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

find_package(Boost REQUIRED)  
find_package(Eigen3 REQUIRED)
find_package(PCL 1.6 REQUIRED )
find_package(OpenGL REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)

add_definitions( ${PCL_DEFINITIONS}
                 -DTW_NO_LIB_PRAGMA
                 -DTW_NO_DIRECT3D
                 -DGLM_FORCE_RADIANS)
                 
find_package(Ceres REQUIRED)

find_library( DIME_LIBS dime )
find_library( GLEW_LIBS GLEW )
find_library( OPENMESHCORE_LIBS OpenMeshCore "/usr/local/lib/OpenMesh" )

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
add_service_files(
  FILES
  volume_in_camera_fov.srv
)

## Generate actions in the 'action' folder
 add_action_files(
   FILES
   RecognizeObject.action
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
#  INCLUDE_DIRS include
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

set(CAM_OBJECT_RECOGNITION_DIR /home/marco/object_recognition)
set(CV_EXT_INCLUDE_DIR ${CAM_OBJECT_RECOGNITION_DIR}/externals/cv_ext/include)
set(CV_EXT_LIBS ${CAM_OBJECT_RECOGNITION_DIR}/externals/cv_ext/lib/libcv_ext.a)

include_directories( 
                      include
                      ${catkin_INCLUDE_DIRS}
                      ${CAM_OBJECT_RECOGNITION_DIR}/include
                      /usr/include
                      /usr/local/include
                      ${CV_EXT_INCLUDE_DIR}
                      ${OpenCV_INCLUDE_DIRS}
                      ${Boost_INCLUDE_DIRS}
                      ${EIGEN3_INCLUDE_DIR}
                      ${PCL_INCLUDE_DIRS}
                      ${CERES_INCLUDE_DIRS}
                      ${OPENGL_INCLUDE_DIRS}
                      ${GLFW_INCLUDE_DIRS})

## Declare a cpp library
# add_library(rockin_obstacle_detection
#   src/${PROJECT_NAME}/rockin_obstacle_detection.cpp
# )

add_definitions(-std=c++11)

## Declare a cpp executable
add_executable(recognition_server src/recognition_server.cpp)
add_executable(arm_camera_object_recognition_multi_server src/arm_camera_object_recognition_multi_server.cpp)
add_executable(arm_camera_object_recognition_server_single_process src/arm_camera_object_recognition_server_single_process.cpp)
add_executable(arm_camera_object_recognition_server src/arm_camera_object_recognition_server.cpp)
add_executable(youbot_object_recognition_client src/youbot_object_recognition_client.cpp)
add_executable(kinect_object_recognition_server src/kinect_object_recognition_server.cpp)
add_executable(kinect_object_recognition_server_rockin src/kinect_object_recognition_server_rockin.cpp)


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(recognition_server youbot_object_recognition_gencpp)
add_dependencies(arm_camera_object_recognition_multi_server youbot_object_recognition_gencpp)
add_dependencies(arm_camera_object_recognition_server youbot_object_recognition_gencpp)
add_dependencies(arm_camera_object_recognition_server_single_process youbot_object_recognition_gencpp)
add_dependencies(kinect_object_recognition_server youbot_object_recognition_gencpp mcr_object_detection_gencpp)
add_dependencies(kinect_object_recognition_server_rockin youbot_object_recognition_gencpp mcr_object_detection_gencpp)
add_dependencies(youbot_object_recognition_client youbot_object_recognition_gencpp mcr_object_detection_gencpp)


## Specify libraries to link a library or executable target against

target_link_libraries( arm_camera_object_recognition_server
                        ${catkin_LIBRARIES})
                        
target_link_libraries( recognition_server
                        ${catkin_LIBRARIES}
                        ${CAM_OBJECT_RECOGNITION_DIR}/lib/libobject_recogntion.a
                       #object_recogntion
                       ${DIME_LIBS}
                       ${CV_EXT_LIBS}
                       ${OpenCV_LIBS}
                       ${PCL_LIBRARIES}
                       ${CERES_LIBRARIES}
                       ${OPENGL_LIBRARY}
                       ${GLEW_LIBS}
                       ${GLFW_STATIC_LIBRARIES}
                       ${OPENMESHCORE_LIBS})
                        
target_link_libraries( arm_camera_object_recognition_multi_server
                        ${catkin_LIBRARIES}
                        ${CAM_OBJECT_RECOGNITION_DIR}/lib/libobject_recogntion.a
                       #object_recogntion
                       ${DIME_LIBS}
                       ${CV_EXT_LIBS}
                       ${OpenCV_LIBS}
                       ${PCL_LIBRARIES}
                       ${CERES_LIBRARIES}
                       ${OPENGL_LIBRARY}
                       ${GLEW_LIBS}
                       ${GLFW_STATIC_LIBRARIES}
                       ${OPENMESHCORE_LIBS})
                      
target_link_libraries( arm_camera_object_recognition_server_single_process
                        ${catkin_LIBRARIES}
                        ${CAM_OBJECT_RECOGNITION_DIR}/lib/libobject_recogntion.a
                       #object_recogntion
                       ${DIME_LIBS}
                       ${CV_EXT_LIBS}
                       ${OpenCV_LIBS}
                       ${PCL_LIBRARIES}
                       ${CERES_LIBRARIES}
                       ${OPENGL_LIBRARY}
                       ${GLEW_LIBS}
                       ${GLFW_STATIC_LIBRARIES}
                       ${OPENMESHCORE_LIBS})


target_link_libraries( kinect_object_recognition_server
                        ${catkin_LIBRARIES}
                        ${PCL_LIBRARIES})
target_link_libraries( youbot_object_recognition_client
                        ${catkin_LIBRARIES})
target_link_libraries( kinect_object_recognition_server_rockin
                        ${catkin_LIBRARIES}
                        ${PCL_LIBRARIES})
                        


