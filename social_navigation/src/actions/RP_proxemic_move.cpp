#include "RP_proxemic_move.h"
#include <unistd.h>
#include <algorithm>

/* constructor */
RP_proxemic_move::RP_proxemic_move(ros::NodeHandle& nh) :
  nh_(nh),
  Action("proxemic_move", 5.0),
  action_client_("/move_base", false),
  obj_listener_("base_footprint"),
  tf_listener_(tfBuffer_),
  cost_map_flood_controller(),
  robot_costmap_ready_(false),
  robot_costmap_(),
  local_costmap_()
{
  srv_goal_ = nh_.serviceClient<topological_navigation_msgs::GetLocation>("/topological_navigation/get_location");
  st_people_goal_ = nh_.serviceClient<social_navigation_layers_msgs::StimatedPeopleGoal>("/proxemic_layer/stimated_people_goal");

  robot_id_ = "sonny";
  default_value_ = 0;
  robot_costmap_sub_ = nh_.subscribe("/social_navigation/robot_costmap", 1, &RP_proxemic_move::robotCostmapCallback, this);
  local_costmap_sub_ = nh_.subscribe("/move_base/local_costmap/costmap", 1, &RP_proxemic_move::localCostmapCallback, this);
  nh_.param("search_radius_", search_radius_, double(1.0));

  /*
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

  obj_listener_.add_class("person", people_conf);*/

}

void RP_proxemic_move::activateCode()
{
  while (!action_client_.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("[proxemic_move] Waiting for the move_base action server to come up");
  }

  std::string wpID, people_goal_id_;
  bool found = false;
  for (size_t i = 0; i < last_msg_.parameters.size(); i++)
  {
    ROS_INFO("Param %s = %s", last_msg_.parameters[i].key.c_str(), last_msg_.parameters[i].value.c_str());
    if (0 == last_msg_.parameters[i].key.compare("robot_goal"))
    {
      wpID_ = last_msg_.parameters[i].value;
      found = true;
    }
    else if (0 == last_msg_.parameters[i].key.compare("people_goal"))
    {
      people_goal_id_ = last_msg_.parameters[i].value;
    }
  }
  std::vector<boost::shared_ptr<geometry_msgs::Pose> > results;
  topological_navigation_msgs::GetLocation srv;
  srv.request.waypoint = people_goal_id_;
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
  social_navigation_layers_msgs::StimatedPeopleGoal social_nav_srv;
  geometry_msgs::Pose p = *(results[0]);
  social_nav_srv.request.target = p.position;
  ROS_INFO("1");
  st_people_goal_.call(social_nav_srv);

  results.clear();
  ROS_INFO("2");

  srv.request.waypoint = wpID_;
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
  ROS_INFO("3");

  findProxemicPose(*(results[0]), goal_pose_);
  ROS_INFO("4");

  goal.target_pose = goal_pose_;
  action_client_.sendGoal(goal);

  //obj_listener_.reset();
  //obj_listener_.set_working_frame("base_footprint");
  //obj_listener_.set_active();
}


void RP_proxemic_move::findProxemicPose(geometry_msgs::Pose target, geometry_msgs::PoseStamped &output_target)
{
  unsigned int mx, my;
  goal_p_.x = target.position.x;
  goal_p_.y = target.position.y;
  local_costmap_.worldToMap(goal_p_.x, goal_p_.y, mx, my);
  flood(mx,my);
  std::sort(proxemic_points_.begin(), proxemic_points_.end(), &RP_proxemic_move::sortProxemicList);
  //for (std::vector<ProxemicPoint>::iterator it = proxemic_points_.begin() ; it != proxemic_points_.end(); ++it)
  //{
  //  ROS_INFO("Point x [%f] y [%f] value [%f]", it->p.x, it->p.y, it->value);
  //}

  ROS_INFO("Result point [%f] y [%f] value [%f]",
   proxemic_points_[0].p.x,
   proxemic_points_[0].p.y,
   proxemic_points_[0].value);

  cost_map_flood_controller.setDefaultValue(0);
  cost_map_flood_controller.resetMap(
    0,
    0,
    cost_map_flood_controller.getSizeInCellsX(),
    cost_map_flood_controller.getSizeInCellsY());
  output_target.pose.position.x = proxemic_points_[0].p.x;
  output_target.pose.position.y = proxemic_points_[0].p.y;
  output_target.header.stamp = ros::Time::now();
  output_target.header.frame_id = "map";

  proxemic_points_.clear();
}

void RP_proxemic_move::deActivateCode()
{
  action_client_.cancelAllGoals();
  //obj_listener_.set_inactive();
}


void RP_proxemic_move::step()
{
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

void RP_proxemic_move::robotCostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
  map2costmap(*map, robot_costmap_);
  robot_costmap_ready_ = true;
}

void RP_proxemic_move::localCostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
  map2costmap(*map, local_costmap_);
  initCostmapFloodControler();
}

void RP_proxemic_move::initCostmapFloodControler()
{
  resolution_ = local_costmap_.getResolution();
  cost_map_flood_controller.resizeMap(
    local_costmap_.getSizeInCellsX(),
    local_costmap_.getSizeInCellsY(),
    local_costmap_.getResolution(),
    local_costmap_.getOriginX(),
    local_costmap_.getOriginY());
  cost_map_flood_controller.setDefaultValue(default_value_);
  cost_map_flood_controller.resetMap(0,0,cost_map_flood_controller.getSizeInCellsX(), cost_map_flood_controller.getSizeInCellsY());
}

void RP_proxemic_move::map2costmap(nav_msgs::OccupancyGrid map, costmap_2d::Costmap2D &costmap)
{
  costmap.resizeMap(map.info.width,map.info.height, map.info.resolution, map.info.origin.position.x, map.info.origin.position.y);
  costmap.setDefaultValue(default_value_);
  costmap.resetMap(0, 0, costmap.getSizeInCellsX(), costmap.getSizeInCellsY());

  for (unsigned int y = 0; y < map.info.height; y++)
    for (unsigned int x = 0; x < map.info.width; x++)
    {
      unsigned int i = (x) + (y) * map.info.width;
      costmap.setCost(x,y,map.data[i] * 254 / 100);
    }
}

bool RP_proxemic_move::cellsOK(unsigned int x, unsigned int y, costmap_2d::Costmap2D &costmap)
{
  return (x < costmap.getSizeInCellsX() &&
    y < costmap.getSizeInCellsY() &&
    x >= 0 &&
    y >= 0);
}

float RP_proxemic_move::point2PointDist(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
  return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

bool RP_proxemic_move::sortProxemicList(ProxemicPoint i,ProxemicPoint j)
{
  return (i.value<j.value);
}

void RP_proxemic_move::flood(unsigned int x,unsigned int y)
{
  float cost_sum = 0;
  if (cellsOK(x, y, local_costmap_))
  {
    if (cost_map_flood_controller.getCost(x,y) != 255 && local_costmap_.getCost(x, y) < 220)
    {
      for(unsigned int cell_y = 0; cell_y < robot_costmap_.getSizeInCellsY(); cell_y++)
        for(unsigned int cell_x = 0; cell_x < robot_costmap_.getSizeInCellsX(); cell_x++)
        {
          //ROS_INFO("cell_y [%u] cell_x[%u], costmap_x[%u], costmap_y[%u]",cell_y,cell_x, x - 10 + cell_x, y - 10 + cell_y);
          unsigned int x_costmap = x - 10, y_costmap = y - 10;
          if (cellsOK(x_costmap + cell_x, y_costmap + cell_y, local_costmap_))
          {
            cost_sum += local_costmap_.getCost(x_costmap + cell_x, y_costmap + cell_y) +
              robot_costmap_.getCost(cell_x, cell_y);
          }
        }
      cost_map_flood_controller.setCost(x,y,255);
      ProxemicPoint proxemic_p;
      local_costmap_.mapToWorld(x, y, proxemic_p.p.x, proxemic_p.p.y);
      if (cost_sum >= 25318 &&
          point2PointDist(goal_p_, proxemic_p.p) <= search_radius_)
      {
        proxemic_p.value = cost_sum;
        proxemic_points_.push_back(proxemic_p);
      }
      flood(x + 1, y);
      flood(x, y + 1);
      flood(x - 1, y);
      flood(x, y - 1);
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rosplan_interface_proxemic_move");
  ros::NodeHandle nh("~");
  std::string actionserver;
  nh.param("action_server", actionserver, std::string("/move_base"));
  RP_proxemic_move rpmb(nh);
  ros::Subscriber ds =
      nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPActionInterface::dispatchCallback,
                   dynamic_cast<KCL_rosplan::RPActionInterface*>(&rpmb));
  rpmb.runActionInterface();
  return 0;
}
