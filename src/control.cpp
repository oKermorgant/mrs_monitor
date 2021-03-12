#include <mrs_monitor/control.h>

using namespace mrs_monitor;

Controller::Controller()
{
  initCostMap();


}

void Controller::initCostMap()
{
  tf2_ros::Buffer buffer(ros::Duration(10));
  tf2_ros::TransformListener tl(buffer);
  ros::NodeHandle costmap_nh("~local_costmap");
  costmap = std::make_unique<costmap_2d::Costmap2DROS>("local_costmap", buffer);
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "controller");

  ros::NodeHandle costmap_nh("/robot1/controller/local_costmap");
  costmap_nh.setParam("robot_radius", 0.2);
  costmap_nh.setParam("global_frame", "robot1/odom");
  costmap_nh.setParam("robot_base_frame", "robot1/base_link");
  costmap_nh.setParam("update_frequency", 5.);
  costmap_nh.setParam("static_map", false);
  costmap_nh.setParam("rolling_window", true);

  Controller control;
  ros::spin();
}
