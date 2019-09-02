/*
 * flood_fill_lib.h
 *
 *  Created on: 21/06/2019
 *      Author: Jonathan Ginés
 */

#ifndef FLOODSEARCHERNODE_H_
#define FLOODSEARCHERNODE_H_

#include <ros/ros.h>
#include <string>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d_publisher.h>

#include <nav_msgs/OccupancyGrid.h>
#include <keep_in_proxemics_msgs/SearchRobotPosition.h>
#include <tf/transform_listener.h>
#include <navfn/navfn_ros.h>



namespace flood_searcher_node {

class FloodSearcher {
public:
  FloodSearcher();

  struct ProxemicPoint
  {
    geometry_msgs::Point p;
    float value;
  };

  void flood(unsigned int x,unsigned int y);

private:
	ros::NodeHandle nh_;
	costmap_2d::Costmap2D cost_map, robot_costmap_, cost_map_flood_controller,
    local_costmap_, person_costmap_;
  ros::Subscriber robot_costmap_sub_, local_costmap_sub_, people_pos_sub_, person_costmap_sub_;
  ros::Publisher simple_goal_pub_;
  ros::ServiceServer flood_service_;
  geometry_msgs::Point goal_p_;
	bool robot_costmap_ready_, global_costmap_ready_;
	unsigned int cells_size_x, cells_size_y;
	double resolution, origin_x ,origin_y, search_radius_;
	unsigned char default_value;
	//costmap_2d::Costmap2DPublisher cost_map_publisher_, global_map_publisher_;
  std::vector<ProxemicPoint> proxemic_points_;
  tf::TransformListener tfListener_;
  geometry_msgs::Point elevator_exit_;
  void robotCostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map);
  void personCostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map);
  void localCostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map);

  bool floodSrv(keep_in_proxemics_msgs::SearchRobotPosition::Request &req,
                keep_in_proxemics_msgs::SearchRobotPosition::Response &res);
  void map2costmap(nav_msgs::OccupancyGrid map, costmap_2d::Costmap2D &costmap);
  bool cellsOK(unsigned int x, unsigned int y, costmap_2d::Costmap2D &costmap);
  static bool sortProxemicList(ProxemicPoint i,ProxemicPoint j);
  void template2costmap(
    unsigned int x,
    unsigned int y,
    costmap_2d::Costmap2D &templ,
    costmap_2d::Costmap2D &costmap,
    int max_value);
  void initCostmapFloodControler();
  float point2PointDist(geometry_msgs::Point p1, geometry_msgs::Point p2);

};

}

#endif /* FLOODSEARCHERNODE_H_ */
