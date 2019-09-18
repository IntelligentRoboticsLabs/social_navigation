#include <sensor_msgs/LaserScan.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>


namespace laser_fake_creator {
class LaserFake {
public:
	LaserFake(): scan(), baseFrameId_("base_footprint")
  {
    scanSub = new message_filters::Subscriber<sensor_msgs::LaserScan> (nh_, "/scan", 5);
		tfScanSub = new tf::MessageFilter<sensor_msgs::LaserScan> (*scanSub, tfListener_, baseFrameId_, 5);
		tfScanSub->registerCallback(boost::bind(&LaserFake::laserCallback, this, _1));
		mock_scan_pub = nh_.advertise<sensor_msgs::LaserScan>("/mock_scan", 1);
    procesing_scan = false;
	}

  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
    if(!procesing_scan)
    {
      procesing_scan = true;
      scan.ranges.resize(scan_in->ranges.size());
      unsigned int n = scan_in->ranges.size();
      for (unsigned int i = 0; i < n; ++i)
        scan.ranges[i] = scan_in->range_max;
      scan.header.frame_id = scan_in->header.frame_id;
      scan.angle_min = scan_in->angle_min;
      scan.angle_max = scan_in->angle_max;
      scan.angle_increment = scan_in->angle_increment;
      scan.time_increment = scan_in->time_increment;
      scan.scan_time = scan_in->scan_time;
      scan.range_min = scan_in->range_min;
      scan.range_max = scan_in->range_max;

      scan.ranges[scan.ranges.size()/2] = 0.5;
    }
  }

  void step()
  {
    scan.header.stamp = ros::Time::now();
    mock_scan_pub.publish(scan);
  }

private:
  ros::NodeHandle nh_;
  tf::TransformListener tfListener_;
  tf::MessageFilter<sensor_msgs::LaserScan>* tfScanSub;
  message_filters::Subscriber<sensor_msgs::LaserScan>* scanSub;
  ros::Publisher mock_scan_pub;
  bool procesing_scan;
  sensor_msgs::LaserScan scan;
  std::string baseFrameId_;

};
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "flood_fill");
  ros::NodeHandle n;
  laser_fake_creator::LaserFake laserFake;
  ros::Rate loop_rate(30);

  while (ros::ok())
  {
    laserFake.step();
    loop_rate.sleep();
    ros::spinOnce();
  }
	return 0;
}
