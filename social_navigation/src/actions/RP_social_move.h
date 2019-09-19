/*********************************************************************
*  Software License Agreement (BSD License)
*
*   Copyright (c) 2018, Intelligent Robotics
*   All rights reserved.
*
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions
*   are met:

*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of Intelligent Robotics nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.

*   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Francisco Martín fmrico@gmail.com */

/* Mantainer: Francisco Martín fmrico@gmail.com */

#include <ros/ros.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <boost/foreach.hpp>

#include <tf2_ros/transform_listener.h>
#include "actionlib/client/simple_action_client.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_srvs/Empty.h"
#include <bica_planning/Action.h>
#include <topological_navigation_msgs/GetLocation.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <bica_graph/graph_client.h>
#include <darknet_ros_3d/Darknet3DListener.h>


#ifndef KCL_social_move
#define KCL_social_move

#define DISTANCE_ROBOT_V_SIZE 3

struct SimpleTrackedPerson
{
  bool is_moving;
  std::vector<float> distance_to_robot;
  float probability;
  tf2::Vector3 last_position_map;
  tf2::Vector3 last_position_bf;
  ros::Time stamp;
  bool updated;
};

#define CHECK_PERSON_CHECKING_TIME   20.0
#define CHECK_PERSON_MIN_PROBABILITY   0.4


#define CHECK_PERSON_MIN_X   0.5
#define CHECK_PERSON_MAX_X   2.0
#define CHECK_PERSON_MIN_Y   -1.0
#define CHECK_PERSON_MAX_Y   1.0
#define CHECK_PERSON_MIN_Z   0.0
#define CHECK_PERSON_MAX_Z   2.0
#define CHECK_PERSON_MIN_SIZE_X   0.15
#define CHECK_PERSON_MIN_SIZE_Y   0.15
#define CHECK_PERSON_MIN_SIZE_Z   0.45
#define CHECK_PERSON_MAX_SIZE_X   1.0
#define CHECK_PERSON_MAX_SIZE_Y   1.0
#define CHECK_PERSON_MAX_SIZE_Z   2.5


class RP_social_move : public bica_planning::Action
{
public:
  explicit RP_social_move(ros::NodeHandle& nh);

protected:
  void activateCode();
  void deActivateCode();
  void step();
  //void timeoutCB(const ros::TimerEvent&);

private:

  ros::NodeHandle nh_;
  enum StateType
  {
    INIT,
    NAVIGATING,
    INTERACTING
  };
  bica_graph::GraphClient graph_;
  std::string actionserver_, robot_id_;
  geometry_msgs::PoseStamped goal_pose_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client_;
  ros::ServiceClient srv_goal_;
  move_base_msgs::MoveBaseGoal goal;
  ros::Subscriber people_sub_;
  StateType state_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tf_listener_;
  bool interaction_achieved, gretting;
  std::vector<SimpleTrackedPerson> people_tracked_list_;
  float distance_th_;
  //ros::Timer timer_interactio;

  ros::Time state_ts_;

  darknet_ros_3d::Darknet3DListener obj_listener_;
};

#endif
