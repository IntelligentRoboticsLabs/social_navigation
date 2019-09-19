cmake_minimum_required(VERSION 2.8.3)
project(social_navigation_layers_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs)

add_service_files(
   FILES
   StimatedPeopleGoal.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
   CATKIN_DEPENDS std_msgs geometry_msgs message_runtime
)
