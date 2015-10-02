cmake_minimum_required(VERSION 2.8.3)
project(youbot_joystick_control)


find_package(catkin REQUIRED COMPONENTS
  #actionlib
  youbot_driver_ros_interface
  brics_actuator
  geometry_msgs
  roscpp
  roslib
  rospy
  std_msgs
  std_srvs
  tf
)
find_package(Boost REQUIRED)
catkin_package(
  CATKIN_DEPENDS
	brics_actuator
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(youbot_joystick_control src/youbot_joystick_control.cpp)
add_executable(youbot_joystick_control_arm_only src/youbot_joystick_control_arm_only.cpp)

add_dependencies(youbot_joystick_control brics_actuator_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(youbot_joystick_control_arm_only brics_actuator_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(youbot_joystick_control ${catkin_LIBRARIES}  ${Boost_LIBS})
target_link_libraries(youbot_joystick_control_arm_only ${catkin_LIBRARIES} )

