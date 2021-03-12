#ifndef MRS_MONITOR_CONTROL_H
#define MRS_MONITOR_CONTROL_H

#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/transform_listener.h>
#include <dwa_local_planner/dwa_planner_ros.h>

namespace mrs_monitor
{

class Controller
{
public:
  Controller();


private:

  std::unique_ptr<costmap_2d::Costmap2DROS> costmap;
  void initCostMap();

  dwa_local_planner::DWAPlannerROS dwa;




};






}

#endif
