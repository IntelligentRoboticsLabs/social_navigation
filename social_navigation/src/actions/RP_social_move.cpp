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

/* Author: Jonatan Gines jonatan.gines@urjc.es */

/* Mantainer: Jonatan Gines jonatan.gines@urjc.es */

#include "RP_social_move.h"

#include <string>
#include <vector>

/* The implementation of RP_social_move.h */

/* constructor */
RP_social_move::RP_social_move(ros::NodeHandle& nh) :
  nh_(nh),
  Action("social_move_to", 5.0),
  action_client_("/move_base", false),
  obj_listener_("base_footprint"),
  tf_listener_(tfBuffer_)
{
  srv_goal_ = nh_.serviceClient<topological_navigation_msgs::GetLocation>("/topological_navigation/get_location");
  interaction_achieved = false;
  gretting = false;
  nh_.param<float>("distance_interaction_threshold", distance_th_, 2.0);
  timer_interaction	=	nh_.createTimer(ros::Duration(20),&RP_social_move::timeoutCB,this,true);
  timer_interaction.stop();
  robot_id_ = "sonny";

  darknet_ros_3d::ObjectConfiguration people_conf;

  people_conf.min_probability = CHECK_PERSON_MIN_PROBABILITY;
  people_conf.min_x = CHECK_PERSON_MIN_X;
  people_conf.max_x = CHECK_PERSON_MAX_X;
  people_conf.min_y = CHECK_PERSON_MIN_Y;
  people_conf.max_y = CHECK_PERSON_MAX_Y;
  people_conf.min_z = CHECK_PERSON_MIN_Z;
  people_conf.max_z = CHECK_PERSON_MAX_Z;
  people_conf.min_size_x = CHECK_PERSON_MIN_SIZE_X;
  people_conf.min_size_y = CHECK_PERSON_MIN_SIZE_Y;
  people_conf.min_size_z = CHECK_PERSON_MIN_SIZE_Z;
  people_conf.max_size_x = CHECK_PERSON_MAX_SIZE_X;
  people_conf.max_size_y = CHECK_PERSON_MAX_SIZE_Y;
  people_conf.max_size_z = CHECK_PERSON_MAX_SIZE_Z;
  people_conf.dynamic = true;

  obj_listener_.add_class("person", people_conf);

}

void RP_social_move::activateCode()
{
  while (!action_client_.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("[social_move_to] Waiting for the move_base action server to come up");
  }

  std::string wpID;
  bool found = false;
  for (size_t i = 0; i < last_msg_.parameters.size(); i++)
  {
    ROS_INFO("Param %s = %s", last_msg_.parameters[i].key.c_str(), last_msg_.parameters[i].value.c_str());
    if (0 == last_msg_.parameters[i].key.compare("to"))
    {
      wpID = last_msg_.parameters[i].value;
      found = true;
    }
  }
  std::vector<boost::shared_ptr<geometry_msgs::Pose> > results;
  topological_navigation_msgs::GetLocation srv;
  srv.request.waypoint = wpID;
  if (srv_goal_.call(srv))
  {
    results.push_back(boost::shared_ptr<geometry_msgs::Pose>(new geometry_msgs::Pose(srv.response.position)));
  }
  else
  {
    ROS_ERROR("Failed to call service /topological_nav/get_location");
    setFail();
    return;
  }

  goal_pose_.pose = *(results[0]);

  ROS_INFO("[social_move_to]Commanding to [%s] (%f %f)",
    wpID.c_str(),
    goal_pose_.pose.position.x,
    goal_pose_.pose.position.y);

  goal.target_pose = goal_pose_;
  goal.target_pose.header.frame_id = "map";
  //goal.target_pose.header.stamp = ros::Time::now();
  //action_client_.sendGoal(goal);
  state_ = INIT;
  graph_.add_edge(robot_id_, "ask: hello.action", robot_id_);

  //obj_listener_.reset();
  //obj_listener_.set_working_frame("base_footprint");
  //obj_listener_.set_active();
}

void RP_social_move::deActivateCode()
{
  action_client_.cancelAllGoals();
  //obj_listener_.set_inactive();
}

void RP_social_move::timeoutCB(const ros::TimerEvent&)
{
  auto interest_edges = graph_.get_string_edges_from_node_by_data(robot_id_, "ask: bye.action");
  if (!interest_edges.empty())
    graph_.remove_edge(interest_edges[0]);

  state_ = INIT;
}

void RP_social_move::step()
{
  //updateTrackedPeopleList();
  //tf2::Vector3 encounter_situation_pos(0,0,0);
  //bool encounter_situation = checkEncounter(encounter_situation_pos);
  switch(state_){
    case INIT:
      ROS_INFO("[social_move] INIT state");
      goal.target_pose.header.stamp = ros::Time::now();
      action_client_.sendGoal(goal);
      state_ = NAVIGATING;
      break;
    case NAVIGATING:
      {
        auto interest_edges = graph_.get_string_edges_from_node_by_data(robot_id_, "response: hello");
        if (!interest_edges.empty())
        {
          graph_.add_edge(robot_id_, "ask: bye.action", robot_id_);
          ROS_ERROR("----------------- ENCOUNTER SITUATION -------------");
          action_client_.cancelAllGoals();
          graph_.remove_edge(interest_edges[0]);
          state_ = INTERACTING;
        }
        break;
      }
    case INTERACTING:
      {
        ROS_INFO("[social_move] INTERACTING state");
        if (!gretting)
        {
          graph_.add_edge(robot_id_, "say: Hi! I'm Sonny and I'm go to the XXX floor.", robot_id_);
          gretting = true;
          timer_interaction.start();
        }
        auto interest_edges = graph_.get_string_edges_from_node_by_data(robot_id_, "response: bye");
        if (!interest_edges.empty())
          state_ = INIT;
        break;
      }
  }

  bool finished_before_timeout = action_client_.waitForResult(ros::Duration(0.5));
  actionlib::SimpleClientGoalState state = action_client_.getState();
  ROS_INFO("KCL: (%s) action state: %s", params.name.c_str(), state.toString().c_str());
  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state = action_client_.getState();
    ROS_INFO("KCL: (%s) action finished: %s", params.name.c_str(), state.toString().c_str());
    if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      setSuccess();
      return;
    }
    else if(state == actionlib::SimpleClientGoalState::ABORTED)
    {
      setFail();
      return;
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rosplan_interface_social_move_to");
  ros::NodeHandle nh("~");

  std::string actionserver;
  nh.param("action_server", actionserver, std::string("/move_base"));

  RP_social_move rpmb(nh);

  ros::Subscriber ds =
      nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPActionInterface::dispatchCallback,
                   dynamic_cast<KCL_rosplan::RPActionInterface*>(&rpmb));
  rpmb.runActionInterface();

  return 0;
}
