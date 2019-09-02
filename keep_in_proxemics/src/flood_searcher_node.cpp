#include "keep_in_proxemics/flood_searcher_node.h"
#include <unistd.h>
#include <algorithm>

namespace flood_searcher_node {

	FloodSearcher::FloodSearcher():
  cost_map_flood_controller(),
  robot_costmap_ready_(false),
  robot_costmap_(),
  local_costmap_(){
		default_value = 0;
    robot_costmap_sub_ = nh_.subscribe("/keep_in_proxemics/robot_costmap", 1, &FloodSearcher::robotCostmapCallback, this);
    local_costmap_sub_ = nh_.subscribe("/move_base/local_costmap/costmap", 1, &FloodSearcher::localCostmapCallback, this);
    simple_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    flood_service_ = nh_.advertiseService("/keep_in_proxemics/search_robot_position", &FloodSearcher::floodSrv, this);
    nh_.param("search_radius_", search_radius_, double(1.0));
	}

  void FloodSearcher::initCostmapFloodControler()
  {
    resolution = local_costmap_.getResolution();
    cost_map_flood_controller.resizeMap(
      local_costmap_.getSizeInCellsX(),
      local_costmap_.getSizeInCellsY(),
      local_costmap_.getResolution(),
      local_costmap_.getOriginX(),
      local_costmap_.getOriginY());
    cost_map_flood_controller.setDefaultValue(default_value);
    cost_map_flood_controller.resetMap(0,0,cost_map_flood_controller.getSizeInCellsX(), cost_map_flood_controller.getSizeInCellsY());
  }

  void FloodSearcher::robotCostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map)
  {
    map2costmap(*map, robot_costmap_);
    robot_costmap_ready_ = true;
  }

  void FloodSearcher::localCostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map)
  {
    map2costmap(*map, local_costmap_);
    initCostmapFloodControler();
  }

  bool FloodSearcher::floodSrv(keep_in_proxemics_msgs::SearchRobotPosition::Request &req,
                          keep_in_proxemics_msgs::SearchRobotPosition::Response &res)
  {
    unsigned int mx, my;
    goal_p_.x = req.target.point.x;
    goal_p_.y = req.target.point.y;
    local_costmap_.worldToMap(goal_p_.x, goal_p_.y, mx, my);
    flood(mx,my);
    std::sort(proxemic_points_.begin(), proxemic_points_.end(), &FloodSearcher::sortProxemicList);
    //for (std::vector<ProxemicPoint>::iterator it = proxemic_points_.begin() ; it != proxemic_points_.end(); ++it)
    //{
    //  ROS_INFO("Point x [%f] y [%f] value [%f]", it->p.x, it->p.y, it->value);
    //}

    ROS_INFO("Result point [%f] y [%f] value [%f]",
     proxemic_points_[0].p.x,
     proxemic_points_[0].p.y,
     proxemic_points_[0].value);
    //template2costmap(proxemic_points_[0].p.x, proxemic_points_[0].p.y, robot_costmap_, cost_map, 254);
    cost_map_flood_controller.setDefaultValue(0);
    cost_map_flood_controller.resetMap(
      0,
      0,
      cost_map_flood_controller.getSizeInCellsX(),
      cost_map_flood_controller.getSizeInCellsY());
    res.result.point.x = proxemic_points_[0].p.x;
    res.result.point.y = proxemic_points_[0].p.y;
    proxemic_points_.clear();
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = "/map";
    msg.header.stamp = ros::Time::now();
    msg.pose.position.x = proxemic_points_[0].p.x;
    msg.pose.position.y = proxemic_points_[0].p.y;
    msg.pose.orientation.w = 1.0;
    simple_goal_pub_.publish(msg);
    return true;
  }

  float FloodSearcher::point2PointDist(geometry_msgs::Point p1, geometry_msgs::Point p2)
  {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
  }
  bool FloodSearcher::sortProxemicList(ProxemicPoint i,ProxemicPoint j)
  {
    return (i.value<j.value);
  }

  void FloodSearcher::map2costmap(nav_msgs::OccupancyGrid map, costmap_2d::Costmap2D &costmap)
  {
    costmap.resizeMap(map.info.width,map.info.height, map.info.resolution, map.info.origin.position.x, map.info.origin.position.y);
    costmap.setDefaultValue(default_value);
    costmap.resetMap(0, 0, costmap.getSizeInCellsX(), costmap.getSizeInCellsY());

		for (unsigned int y = 0; y < map.info.height; y++)
			for (unsigned int x = 0; x < map.info.width; x++)
      {
				unsigned int i = (x) + (y) * map.info.width;
				costmap.setCost(x,y,map.data[i] * 254 / 100);
			}
	}

  void FloodSearcher::template2costmap(
    unsigned int x,
    unsigned int y,
    costmap_2d::Costmap2D &templ,
    costmap_2d::Costmap2D &costmap,
    int max_value)
  {
    for(unsigned int cell_y = 0; cell_y < templ.getSizeInCellsY(); cell_y++)
      for(unsigned int cell_x = 0; cell_x < templ.getSizeInCellsX(); cell_x++)
      {
        unsigned int x_costmap = x - templ.getSizeInCellsX()/2, y_costmap = y - templ.getSizeInCellsY()/2;
        if (cellsOK(x_costmap + cell_x, y_costmap + cell_y, costmap))
        {
          int value = templ.getCost(cell_x, cell_y) +
           costmap.getCost(x_costmap + cell_x, y_costmap + cell_y);
          if (value > max_value)
            value = max_value;
          costmap.setCost(x_costmap + cell_x, y_costmap + cell_y, value);
        }
      }

  }

  bool FloodSearcher::cellsOK(unsigned int x, unsigned int y, costmap_2d::Costmap2D &costmap)
  {
		return (x < costmap.getSizeInCellsX() &&
      y < costmap.getSizeInCellsY() &&
      x >= 0 &&
      y >= 0);
	}

  void FloodSearcher::flood(unsigned int x,unsigned int y)
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
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "flood_searcher_node");
  ros::NodeHandle n;
  flood_searcher_node::FloodSearcher floodFill;
  ros::Rate loop_rate(10);
	while (ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
