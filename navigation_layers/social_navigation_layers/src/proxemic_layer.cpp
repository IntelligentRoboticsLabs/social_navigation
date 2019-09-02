#include <social_navigation_layers/proxemic_layer.h>
#include <math.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(social_navigation_layers::ProxemicLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

double gaussian(double x, double y, double x0, double y0, double A, double varx, double vary, double skew){
    double dx = x-x0, dy = y-y0;
    double h = sqrt(dx*dx+dy*dy);
    double angle = atan2(dy,dx);
    double mx = cos(angle-skew) * h;
    double my = sin(angle-skew) * h;
    double f1 = pow(mx, 2.0)/(2.0 * varx),
           f2 = pow(my, 2.0)/(2.0 * vary);
    return A * exp(-(f1 + f2));
}

double get_radius(double cutoff, double A, double var){
    return sqrt(-2*var * log(cutoff/A) );
}


namespace social_navigation_layers
{
    void ProxemicLayer::onInitialize()
    {
        SocialLayer::onInitialize();
        ros::NodeHandle nh("~/" + name_), g_nh;
        server_ = new dynamic_reconfigure::Server<ProxemicLayerConfig>(nh);
        f_ = boost::bind(&ProxemicLayer::configure, this, _1, _2);
        server_->setCallback(f_);
    }

    void ProxemicLayer::updateBoundsFromPeople(double* min_x, double* min_y, double* max_x, double* max_y)
    {
        std::list<people_msgs::Person>::iterator p_it;

        for(p_it = transformed_people_.begin(); p_it != transformed_people_.end(); ++p_it){
            people_msgs::Person person = *p_it;

            double mag = sqrt(pow(person.velocity.x,2) + pow(person.velocity.y, 2));
            double factor = 1.0 + mag * factor_;
            double point = get_radius(cutoff_, amplitude_, covar_ * factor );

            *min_x = std::min(*min_x, person.position.x - point);
            *min_y = std::min(*min_y, person.position.y - point);
            *max_x = std::max(*max_x, person.position.x + point);
            *max_y = std::max(*max_y, person.position.y + point);

        }
    }

    void ProxemicLayer::setCost(
      people_msgs::Person person,
      costmap_2d::Costmap2D* costmap,
      double res,
      int min_i,
      int min_j,
      int max_i,
      int max_j)
    {
      double angle = atan2(person.velocity.y, person.velocity.x);
      double mag = sqrt(pow(person.velocity.x,2) + pow(person.velocity.y, 2));
      double factor = 1.0 + mag * factor_;
      double base = get_radius(cutoff_, amplitude_, covar_);
      double point = get_radius(cutoff_, amplitude_, covar_ * factor );

      unsigned int width = std::max(1, int( (base + point) / res )),
                    height = std::max(1, int( (base + point) / res ));

      double cx = person.position.x, cy = person.position.y;

      double ox, oy;
      if(sin(angle)>0)
          oy = cy - base;
      else
          oy = cy + (point-base) * sin(angle) - base;

      if(cos(angle)>=0)
          ox = cx - base;
      else
          ox = cx + (point-base) * cos(angle) - base;


      int dx, dy;
      costmap->worldToMapNoBounds(ox, oy, dx, dy);

      int start_x = 0, start_y=0, end_x=width, end_y = height;
      if(dx < 0)
          start_x = -dx;
      else if(dx + width > costmap->getSizeInCellsX())
          end_x = std::max(0, (int)costmap->getSizeInCellsX() - dx);

      if((int)(start_x+dx) < min_i)
          start_x = min_i - dx;
      if((int)(end_x+dx) > max_i)
          end_x = max_i - dx;

      if(dy < 0)
          start_y = -dy;
      else if(dy + height > costmap->getSizeInCellsY())
          end_y = std::max(0, (int) costmap->getSizeInCellsY() - dy);

      if((int)(start_y+dy) < min_j)
          start_y = min_j - dy;
      if((int)(end_y+dy) > max_j)
          end_y = max_j - dy;

      double bx = ox + res / 2,
             by = oy + res / 2;
      for(int i=start_x;i<end_x;i++){
          for(int j=start_y;j<end_y;j++){
            unsigned char old_cost = costmap->getCost(i+dx, j+dy);
            if(old_cost == costmap_2d::NO_INFORMATION)
              continue;

            double x = bx+i*res, y = by+j*res;
            double ma = atan2(y-cy,x-cx);
            double diff = angles::shortest_angular_distance(angle, ma);
            double a;
            if(fabs(diff)<M_PI/2)
                a = gaussian(x,y,cx,cy,amplitude_,covar_*factor,covar_,angle);
            else
                a = gaussian(x,y,cx,cy,amplitude_,covar_,       covar_,0);

            if(a < cutoff_)
              continue;
            unsigned char cvalue = (unsigned char) a;
            costmap->setCost(i+dx, j+dy, std::max(cvalue, old_cost));

        }
      }
    }

    bool ProxemicLayer::getPlan(
      costmap_2d::Costmap2D* costmap_,
      geometry_msgs::Point start,
      geometry_msgs::Point goal,
      std::vector <geometry_msgs::PoseStamped> &plan)
    {
      std::string global_frame = "map";
      navfn::NavfnROS navfn;
      navfn.initialize("my_navfn_planner", costmap_, global_frame);
      geometry_msgs::PoseStamped p_init, p_goal;
      p_init.header.frame_id = global_frame;
      p_init.pose.position.x = start.x;
      p_init.pose.position.y = start.y;
      p_init.pose.orientation.w = 1.0;

      p_goal.header.frame_id = global_frame;
      p_goal.pose.position.x = goal.x;
      p_goal.pose.position.y = goal.y;
      p_goal.pose.orientation.w = 1.0;

      navfn.makePlan(p_init, p_goal, plan);
      return plan.size() > 0;
    }

    void ProxemicLayer::resamplingPlan(
      std::vector<geometry_msgs::PoseStamped> &plan,
      std::vector<geometry_msgs::PoseStamped> &plan_sampled)
    {
      int count = 0;
      for(auto it = plan.begin(); it != plan.end(); ++it)
      {
        count++;
        if (count == 5)
        {
          plan_sampled.push_back(*it);
          count = 0;
        }
      }
    }

    void ProxemicLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j){
        boost::recursive_mutex::scoped_lock lock(lock_);
        if(!enabled_) return;

        if( people_list_.people.size() == 0 )
          return;
        if( cutoff_ >= amplitude_)
            return;

        costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
        double res = costmap->getResolution();

        costmap_to_plan = new costmap_2d::Costmap2D(
          costmap->getSizeInCellsX(),
          costmap->getSizeInCellsY(),
          res,
          costmap->getOriginX(),
          costmap->getOriginY(),
          0);

        for(auto p_it = transformed_people_.begin(); p_it != transformed_people_.end(); ++p_it)
        {
          TrackedPerson p;
          std::vector <geometry_msgs::PoseStamped> plan, plan_sampled;

          p.person = *p_it;
          geometry_msgs::Point p_point = p_it->position;
          geometry_msgs::Point p_exit_;
          p_exit_.x = exit_px_;
          p_exit_.y = exit_py_;
          if (getPlan(costmap_to_plan, p_point, p_exit_, plan))
          {
            resamplingPlan(plan, plan_sampled);
            p.plan = plan_sampled;
            tracked_person_list_.push_back(p);
          }
          else
            ROS_ERROR("Path for person doesn't found");
        }

        for(auto p_tracked_it = tracked_person_list_.begin();
          p_tracked_it != tracked_person_list_.end();
          ++p_tracked_it)
        {
          geometry_msgs::Point p_point = p_tracked_it->person.position;
          //ROS_INFO("----------------------------------");
          //ROS_INFO("%f, %f", p_point.x, p_point.y);
          //ROS_INFO("cutoff_ %f", cutoff_);
          //ROS_INFO("amplitude_ %f", amplitude_);
          //ROS_INFO("factor_ %f", factor_);
          //ROS_INFO("covar_ %f", covar_);
          float default_amplitude = amplitude_;
          for (auto plan_it = p_tracked_it->plan.begin();
            plan_it != p_tracked_it->plan.end();
            ++plan_it)
          {
            people_msgs::Person p_fake;
            p_fake.position = plan_it->pose.position;
            amplitude_ = exit_path_amplitude_;
            setCost(p_fake, costmap, res, min_i, min_j, max_i, max_j);
          }
          amplitude_ = default_amplitude;
          setCost(p_tracked_it->person, costmap, res, min_i, min_j, max_i, max_j);
        }
        tracked_person_list_.clear();
        people_list_.people.clear();
    }

    void ProxemicLayer::configure(ProxemicLayerConfig &config, uint32_t level) {
        cutoff_ = config.cutoff;
        amplitude_ = config.amplitude;
        covar_ = config.covariance;
        factor_ = config.factor;
        people_keep_time_ = ros::Duration(config.keep_time);
        enabled_ = config.enabled;
        exit_px_ = config.exit_px;
        exit_py_ = config.exit_py;
        exit_path_amplitude_ = config.exit_path_amplitude;

    }


};
