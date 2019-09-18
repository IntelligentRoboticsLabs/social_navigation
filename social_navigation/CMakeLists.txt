cmake_minimum_required(VERSION 2.8.3)
project(social_navigation)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  bica_planning
  bica_graph
  tf
  geometry_msgs
  nav_msgs
  visualization_msgs
  bica
  move_base_msgs
  sensor_msgs
  std_msgs
  cv_bridge
  costmap_2d
  image_transport
  topological_navigation
  topological_navigation_msgs
  darknet_ros_3d
  roslint
)
find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
#    INCLUDE_DIRS include
#    LIBRARIES vizbox
    CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs
#    DEPENDS system_lib
)

include_directories(
# include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

set(ROSLINT_CPP_OPTS "--filter=-build/header_guard,-runtime/references")
roslint_cpp()

# Utils

add_executable(laser_fake_creator_node src/utils/laser_fake_creator_node.cpp)
target_link_libraries(laser_fake_creator_node ${catkin_LIBRARIES})

add_executable(people_remapper_node src/utils/people_remapper_node.cpp)
target_link_libraries(people_remapper_node ${catkin_LIBRARIES})

# Actions

add_executable(rp_social_move src/actions/RP_social_move.cpp)
add_dependencies(rp_social_move ${catkin_EXPORTED_TARGETS} )
target_link_libraries(rp_social_move ${catkin_LIBRARIES} )

add_executable(rp_proxemic_move src/actions/RP_proxemic_move.cpp)
add_dependencies(rp_proxemic_move ${catkin_EXPORTED_TARGETS} )
target_link_libraries(rp_proxemic_move ${catkin_LIBRARIES} )


## Mark executables and/or libraries for installation
install(TARGETS
    rp_social_move
    rp_proxemic_move
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch/
)

install(DIRECTORY pddl/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/pddl/
)

#############
## Testing ##
#############

#if(CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
#
#  add_rostest_gtest(${PROJECT_NAME}-test test/test_PersonNav.launch test/test_PersonNav.cpp)
#
#  target_link_libraries(
#    ${PROJECT_NAME}-test
#    ${catkin_LIBRARIES}
#  )
#
#endif()
