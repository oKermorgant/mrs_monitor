#include <mrs_monitor/control.h>

// local planners
#include <dwa_local_planner/dwa_planner_ros.h>
#include <mrs_monitor/greedy_control.h>

using namespace mrs_monitor;

Controller::Controller() : priv("~"), buffer(ros::Duration(10)), tl(buffer)
{
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  // emulate move_base_simple/goal  
  goal_sub = nh.subscribe("move_base_simple/goal", 10, &Controller::goalCallback, this);
  plan_srv = nh.serviceClient<nav_msgs::GetPlan>("/mrs_planner/GlobalPlanner/make_plan");
  plan_sub = nh.subscribe("move_base_simple/plan", 10, &Controller::beginTrackingPath, this);

  // emulate move_base/status @ control freq
  status_pub = priv.advertise<actionlib_msgs::GoalStatusArray>("status", 10);
  refresh_still.fromSec(1);  
  refresh_moving.fromSec(1./priv.param("controller_frequency", 5.));
  timer = nh.createTimer(refresh_still, [&](auto &){refresh();});
}

void Controller::refresh()
{
  static actionlib_msgs::GoalStatusArray goal_status;
  goal_status.status_list.resize(1);

  cmd_vel.linear.x = cmd_vel.linear.y = cmd_vel.linear.z =
      cmd_vel.angular.x = cmd_vel.angular.y = cmd_vel.angular.z = 0;

  if(status == Status::MOVING &&
     (local_planner->isGoalReached() || !local_planner->computeVelocityCommands(cmd_vel)))
    {
      status = Status::DONE;
      timer.setPeriod(refresh_still);
    }

  cmd_vel_pub.publish(cmd_vel);

  goal_status.status_list.back().status =
      status == Status::DONE ? actionlib_msgs::GoalStatus::PENDING : actionlib_msgs::GoalStatus::ACTIVE;

  goal_status.header.stamp = ros::Time::now();
  status_pub.publish(goal_status);
}

void Controller::goalCallback(const geometry_msgs::PoseStampedConstPtr &goal)
{
  // to service
  static nav_msgs::GetPlan plan;

  // get pose of robot in map frame
  convert(buffer.lookupTransform("map", costmap->getBaseFrameID(), ros::Time(0)),
          plan.request.start);
  plan.request.goal = *goal;

  if(!plan_srv.call(plan) || plan.response.plan.poses.size() == 0)
    return;

  beginTrackingPath(plan.response.plan);
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_base");  

  Controller control;

  // for greedy: set small local costmap
  control.setLocalParam("local_costmap/width", .5);
  control.setLocalParam("local_costmap/height", .5);
  control.initLocalPlanner<GreedyNoWalls>();

  // control.initLocalPlanner<dwa_local_planner::DWAPlannerROS>();

  ros::spin();
}
