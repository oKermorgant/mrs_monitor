#include <ros/ros.h>
#include <mrs_monitor/fleet.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mrs_monitor");
  mrs_monitor::Fleet fleet;

  ros::spin();
}
