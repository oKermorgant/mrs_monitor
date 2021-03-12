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

  plan_srv = nh.serviceClient<nav_msgs::GetPlan>("/mrs_planner/planner/make_plan");
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
  static nav_msgs::GetPlan plan_IO = []()
  {nav_msgs::GetPlan plan;
    plan.request.start.header.frame_id = plan.request.goal.header.frame_id = "map";
    return plan;
  }();

  convert(req.start, plan_IO.request.start.pose);
  convert(req.goal, plan_IO.request.goal.pose);

  if(plan_srv.call(plan_IO) && plan_IO.response.plan.poses.size())
  {
    // estimate travel time for this path
    res.seconds = travelTime(plan_IO.response.plan.poses, req.v_max, req.w_max);
  }
  else
    res.seconds = -1;
  return true;
}

bool Fleet::moveCallback(MoveRequest &req, MoveResponse &)
{
  auto robot = getRobot(req.robot_name);

  if(robot->move(req.goal, req.v_max, req.w_max))
  {
    // also update goals
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
