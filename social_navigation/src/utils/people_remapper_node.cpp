#include "ros/ros.h"
#include "people_msgs/People.h"
#include "people_msgs/PositionMeasurementArray.h"
#include <darknet_ros_3d/Darknet3DListener.h>

#define CHECK_WAITING_PERSON_MIN_PROBABILITY   0.4

#define CHECK_WAITING_PERSON_MIN_X   -1.5
#define CHECK_WAITING_PERSON_MAX_X   1.5
#define CHECK_WAITING_PERSON_MIN_Y   -1.5
#define CHECK_WAITING_PERSON_MAX_Y   1.5
#define CHECK_WAITING_PERSON_MIN_Z   0.0
#define CHECK_WAITING_PERSON_MAX_Z   2.0
#define CHECK_WAITING_PERSON_MIN_SIZE_X   0.15
#define CHECK_WAITING_PERSON_MIN_SIZE_Y   0.15
#define CHECK_WAITING_PERSON_MIN_SIZE_Z   0.45
#define CHECK_WAITING_PERSON_MAX_SIZE_X   1.0
#define CHECK_WAITING_PERSON_MAX_SIZE_Y   1.0
#define CHECK_WAITING_PERSON_MAX_SIZE_Z   2.5

class PeopleRemaper
{
public:
	PeopleRemaper(): nh_(), obj_listener_("map")

	{
    people_pub_ = nh_.advertise<people_msgs::People>("/people", 1);
    nh_.param("frame_id", frame_id_, std::string("odom"));

    darknet_ros_3d::ObjectConfiguration person_conf;

    person_conf.min_probability = CHECK_WAITING_PERSON_MIN_PROBABILITY;
    person_conf.min_x = CHECK_WAITING_PERSON_MIN_X;
    person_conf.max_x = CHECK_WAITING_PERSON_MAX_X;
    person_conf.min_y = CHECK_WAITING_PERSON_MIN_Y;
    person_conf.max_y = CHECK_WAITING_PERSON_MAX_Y;
    person_conf.min_z = CHECK_WAITING_PERSON_MIN_Z;
    person_conf.max_z = CHECK_WAITING_PERSON_MAX_Z;
    person_conf.min_size_x = CHECK_WAITING_PERSON_MIN_SIZE_X;
    person_conf.min_size_y = CHECK_WAITING_PERSON_MIN_SIZE_X;
    person_conf.min_size_z = CHECK_WAITING_PERSON_MIN_SIZE_Z;
    person_conf.max_size_x = CHECK_WAITING_PERSON_MAX_SIZE_X;
    person_conf.max_size_y = CHECK_WAITING_PERSON_MAX_SIZE_Y;
    person_conf.max_size_z = CHECK_WAITING_PERSON_MAX_SIZE_Z;
    person_conf.dynamic = true;

    obj_listener_.add_class("person", person_conf);
    obj_listener_.reset();
    obj_listener_.set_working_frame(frame_id_);
    obj_listener_.set_active();
	}

  void step()
  {
    people_msgs::People people_msg;
    people_msg.header.frame_id = frame_id_;
    std::vector<people_msgs::Person> p_list;

    for (const auto& object : obj_listener_.get_objects())
    {
      people_msgs::Person p;
      geometry_msgs::Point pos, vel;
      pos.x = object.central_point.getX();
      pos.y = object.central_point.getY();
      vel.x = object.speed.getX();
      vel.y = object.speed.getY();
      p.position = pos;
      p.velocity = vel;
      p_list.push_back(p);
    }

    people_msg.people = p_list;
    people_pub_.publish(people_msg);
  }

private:
	ros::NodeHandle nh_;
  ros::Publisher people_pub_;
  std::string frame_id_;
  darknet_ros_3d::Darknet3DListener obj_listener_;
};

int main(int argc, char **argv)
{
   ros::init(argc, argv, "people_remapper");
   ros::NodeHandle n;
   PeopleRemaper people_pub;
   ros::Rate loop_rate(10);
   while (ros::ok())
   {
     people_pub.step();
     ros::spinOnce();
     loop_rate.sleep();
   }
   return 0;

 }
