#include <mrs_monitor/planner.h>

using namespace mrs_monitor;

Planner::Planner() : costmap(buildCostMap())
{
  planner.initialize("planner", &costmap, "map");
}

costmap_2d::Costmap2D Planner::buildCostMap()
{
  tf2_ros::Buffer buffer(ros::Duration(10));
  tf2_ros::TransformListener tl(buffer);

  ros::NodeHandle costmap_nh("~global_costmap");
  costmap_nh.setParam("global_frame", "map");
  costmap_nh.setParam("robot_base_frame", "map");
  costmap_nh.setParam("static_map", true);
  costmap_2d::Costmap2DROS costmap("global_costmap", buffer);
  ros::Rate rate(0.1);

  while(!costmap.getCostmap()->getSizeInCellsX())
  {
    ros::spinOnce();
    rate.sleep();
  }
  return *costmap.getCostmap();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "global_planner");

  Planner planner;
  ros::spin();
}
