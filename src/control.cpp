#include <mrs_monitor/control.h>
#include <nav_msgs/GetPlan.h>

using namespace mrs_monitor;

Controller::Controller() : buffer(ros::Duration(10)), tl(buffer), costmap("local_costmap", buffer)
{
  dwa.initialize("DWAPlannerROS", &buffer, &costmap);

  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  // emulate move_base_simple/goal  
  goal_sub = nh.subscribe("move_base_simple/goal", 10, &Controller::goalCallback, this);
  initPlanningSrv();
  plan_sub = nh.subscribe("move_base_simple/plan", 10, &Controller::beginTrackingPath, this);

  // emulate move_base/status @ control freq
  ros::NodeHandle priv("~");
  status_pub = priv.advertise<actionlib_msgs::GoalStatusArray>("status", 10);
  refresh_still.fromSec(1);  
  refresh_moving.fromSec(1./priv.param("controller_frequency", 5.));
  timer = nh.createTimer(refresh_still, [&](auto &){refresh();});
}

void Controller::initPlanningSrv()
{
  // find a service with make_plan
  // https://answers.ros.org/question/151611/rosservice-list-and-info-from-c/
  XmlRpc::XmlRpcValue req = "/node";
  XmlRpc::XmlRpcValue res;
  XmlRpc::XmlRpcValue pay;
  ros::master::execute("getSystemState",req ,res ,pay ,true);
  std::string plan_srv_name;

  ros::Rate rate(1.);
  while(ros::ok() && plan_srv_name == "")
  {
    ROS_INFO("Looking for a make_plan service...");

    for(int x=0 ; x<res[2][2].size() ; x++)
    {
      std::string gh = res[2][2][x][0].toXml().c_str();
      gh.erase(gh.begin(), gh.begin()+7);
      gh.erase(gh.end()-8, gh.end());
      if(gh.size() > 9 && gh.substr(gh.size()-9, gh.npos) == "make_plan")
      {
        plan_srv_name = gh;
        break;
      }
    }
    rate.sleep();
  }
  if(plan_srv_name.size())
  {
    ROS_INFO("Ready to make_plan!");
    plan_srv = nh.serviceClient<nav_msgs::GetPlan>(plan_srv_name);
  }
  else
    ros::shutdown();
}

void Controller::refresh()
{
  static actionlib_msgs::GoalStatusArray goal_status;
  goal_status.status_list.resize(1);

  cmd_vel.linear.x = cmd_vel.linear.y = cmd_vel.linear.z =
      cmd_vel.angular.x = cmd_vel.angular.y = cmd_vel.angular.z = 0;

  if(status == Status::MOVING &&
     (dwa.isGoalReached() || !dwa.computeVelocityCommands(cmd_vel)))
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
  convert(buffer.lookupTransform("map", costmap.getBaseFrameID(), ros::Time(0)),
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
  ros::spin();
}
