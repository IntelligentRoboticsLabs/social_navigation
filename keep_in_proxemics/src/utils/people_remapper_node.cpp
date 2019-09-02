#include "ros/ros.h"
#include "people_msgs/People.h"
#include "people_msgs/PositionMeasurementArray.h"
#include "tf/transform_listener.h"

class PeopleRemaper
{
public:
	PeopleRemaper(): nh_()
	{
		face_sub_ = nh_.subscribe("/face_detector/people_tracker_measurements_array", 1, &PeopleRemaper::messageCallback, this);
    people_pub_ = nh_.advertise<people_msgs::People>("/people", 1);
    nh_.param("frame_id", frame_id_, std::string("odom"));
	}

  void messageCallback(const people_msgs::PositionMeasurementArray::ConstPtr& msg)
  {
    people_msgs::People people_msg;
    people_msg.header.frame_id = frame_id_;
    std::vector<people_msgs::Person> p_list;
    for (auto it = msg->people.begin(); it != msg->people.end(); ++it)
    {
      people_msgs::Person p;
      geometry_msgs::PointStamped stamped_in, stamped_out;
      stamped_in.header.frame_id = it->header.frame_id;
      stamped_in.point = it->pos;
      tf_.transformPoint(people_msg.header.frame_id, stamped_in, stamped_out);

      p.position = stamped_out.point;
      p_list.push_back(p);
    }

    people_msg.people = p_list;
    people_pub_.publish(people_msg);

  }

private:
	ros::NodeHandle nh_;
	ros::Subscriber face_sub_;
  ros::Publisher people_pub_;
  tf::TransformListener tf_;
  std::string frame_id_;
};

int main(int argc, char **argv)
{
   ros::init(argc, argv, "people_remapper");
   ros::NodeHandle n;

   PeopleRemaper people_pub;
   ros::spin();
   return 0;

 }
