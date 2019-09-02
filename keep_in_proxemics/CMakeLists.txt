cmake_minimum_required(VERSION 2.8.3)
project(keep_in_proxemics)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  costmap_2d
  message_filters
  sensor_msgs
  keep_in_proxemics_msgs
  navfn
  people_msgs
)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
CATKIN_DEPENDS roscpp
)

include_directories(
  INCLUDE_DIRS include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(flood_searcher_node src/flood_searcher_node.cpp)
target_link_libraries(flood_searcher_node ${catkin_LIBRARIES})

add_executable(laser_fake_creator_node src/utils/laser_fake_creator_node.cpp)
target_link_libraries(laser_fake_creator_node ${catkin_LIBRARIES})

add_executable(people_remapper_node src/utils/people_remapper_node.cpp)
target_link_libraries(people_remapper_node ${catkin_LIBRARIES})
