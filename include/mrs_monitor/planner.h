#ifndef MRS_MONITOR_PLANNER_H
#define MRS_MONITOR_PLANNER_H

#include <global_planner/planner_core.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/transform_listener.h>

namespace mrs_monitor
{
class Planner
{
public:
  Planner(tf2_ros::Buffer &buffer);
private:
  ros::NodeHandle nh;

  costmap_2d::Costmap2DROS costmap;


  global_planner::GlobalPlanner planner;
};
}

#endif
