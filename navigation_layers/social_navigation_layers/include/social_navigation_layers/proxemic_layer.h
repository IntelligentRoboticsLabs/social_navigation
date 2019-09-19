#ifndef PROXEMIC_LAYER_H_
#define PROXEMIC_LAYER_H_
#include <ros/ros.h>
#include <social_navigation_layers/social_layer.h>
#include <dynamic_reconfigure/server.h>
#include <social_navigation_layers/ProxemicLayerConfig.h>
#include <social_navigation_layers_msgs/StimatedPeopleGoal.h>


#include <navfn/navfn_ros.h>


double gaussian(double x, double y, double x0, double y0, double A, double varx, double vary, double skew);
double get_radius(double cutoff, double A, double var);

namespace social_navigation_layers
{
  class ProxemicLayer : public SocialLayer
  {
    public:
      ProxemicLayer() { layered_costmap_ = NULL; }

      struct TrackedPerson
      {
        people_msgs::Person person;
        std::vector<geometry_msgs::PoseStamped> plan;
      };

      virtual void onInitialize();
      virtual void updateBoundsFromPeople(double* min_x, double* min_y, double* max_x, double* max_y);
      virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

    protected:
      void configure(ProxemicLayerConfig &config, uint32_t level);
      void setCost(
        people_msgs::Person person,
        costmap_2d::Costmap2D* costmap,
        double res,
        int min_i,
        int min_j,
        int max_i,
        int max_j);
      bool getPlan(
        costmap_2d::Costmap2D* costmap_,
        geometry_msgs::Point start,
        geometry_msgs::Point goal,
        std::vector<geometry_msgs::PoseStamped> &plan);
      void resamplingPlan(
        std::vector<geometry_msgs::PoseStamped> &plan,
        std::vector<geometry_msgs::PoseStamped> &plan_sampled);
      bool stimatedPeopleGoalSrv(
        social_navigation_layers_msgs::StimatedPeopleGoal::Request  &req,
        social_navigation_layers_msgs::StimatedPeopleGoal::Response &res);
      double cutoff_, amplitude_, covar_, factor_, exit_px_, exit_py_,
        exit_path_amplitude_;
      dynamic_reconfigure::Server<ProxemicLayerConfig>* server_;
      dynamic_reconfigure::Server<ProxemicLayerConfig>::CallbackType f_;
      std::vector<TrackedPerson> tracked_person_list_;
      costmap_2d::Costmap2D* costmap_to_plan;
      ros::ServiceServer st_people_goal_;


  };
};


#endif
