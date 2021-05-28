#include <mrs_monitor/fleet.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseArray.h>

using namespace mrs_monitor;

Fleet::Fleet() : nh("~"), tf_listener(tf_buffer)
{
  Robot::listenThrough(tf_buffer);

  estimate_srv = nh.advertiseService("estimate", &Fleet::estimateCallback, this);
  move_srv = nh.advertiseService("move", &Fleet::moveCallback, this);
  status_srv = nh.advertiseService("status", &Fleet::statusCallback, this);

  goals_pub = nh.advertise<geometry_msgs::PoseArray>("last_goals", 10);

  plan_srv = nh.serviceClient<nav_msgs::GetPlan>("/mrs_planner/GlobalPlanner/make_plan");
  last_plan.request.start.header.frame_id = last_plan.request.goal.header.frame_id = "map";
}

Robot* Fleet::getRobot(const std::string &name)
{
  auto robot = std::find_if(fleet.begin(), fleet.end(), [&](const auto &robot)
  {return robot->is(name);});

  if(robot != fleet.end())
    return robot->get();

  // add this robot
  ROS_INFO("Monitoring new robot: %s", name.c_str());
  fleet.emplace_back(std::make_unique<Robot>(nh, name));
  last_paths[fleet.back().get()] = {};

  return fleet.back().get();
}

bool Fleet::estimateCallback(EstimateRequest &req, EstimateResponse &res)
{  
  if(!plan_srv.exists())
  {
    ROS_WARN("Please run the global planner in order to estimate plans");
    return false;
  }

  // get path
  convert(req.start, last_plan.request.start.pose);
  convert(req.goal, last_plan.request.goal.pose);

  if(plan_srv.call(last_plan) && last_plan.response.plan.poses.size())
  {
    // estimate travel time for this path, the function requires inverse of max vels as inputs
    res.seconds = travelTime(last_plan.response.plan.poses, 1.0/req.v_max, 1.0/req.w_max);

    // register if it was the path from one robot pose
    for(auto &[robot,path]: last_paths)
    {
      if(robot->timeTo(last_plan.request.start) < 0.01)
        path = last_plan.response.plan;
    }
  }
  else
    res.seconds = -1;
  return true;
}

bool Fleet::moveCallback(MoveRequest &req, MoveResponse &)
{
  auto robot = getRobot(req.robot_name);


  // find this path before asking to track it
  convert(req.goal, last_plan.request.goal.pose);

  if(robot->timeTo(last_plan.request.goal) < 0.01)
  {
    // we have already found this path
    robot->setPath(last_paths[robot]);
  }
  else
  {
    robot->getPose(last_plan.request.start);
    plan_srv.call(last_plan);
    robot->setPath(last_plan.response.plan);
  }

  if(robot->trackGoal(req.goal, req.v_max, req.w_max))
  {
    // also update all robot goals for display
    static geometry_msgs::PoseArray poses;
    poses.poses.clear();
    poses.header.frame_id = "map";
    poses.header.stamp = ros::Time::now();
    for(const auto &robot: fleet)
      poses.poses.push_back(robot->lastGoal());
    goals_pub.publish(poses);

    return true;
  }
  return false;
}

bool Fleet::statusCallback(StatusRequest &req, StatusResponse &res)
{
  auto robot = getRobot(req.robot_name);

  robot->getPose(res.pose);
  res.seconds_to_goal = robot->timeToGoal();
  return true;
}

