#include <mrs_monitor/monitor_io.h>
#include <nav_msgs/GetPlan.h>

using namespace mrs_monitor;

ros::NodeHandle* Robot_IO::nh;

Monitor_IO::Monitor_IO(ros::NodeHandle &nh, std::string ns) : nh(nh)
{
  Robot_IO::nh = &nh;
  ns = "/" + ns + "/";
  // setup service callers and their messages
  estimate_srv = nh.serviceClient<Estimate>(ns + "estimate");
  move_srv = nh.serviceClient<Move>(ns + "move");
  status_srv = nh.serviceClient<Status>(ns + "status");
}

bool Monitor_IO::exists(std::string name) const
{
  return nh.hasParam("/" + name + "/robot_description");
}

double Monitor_IO::estimate(const geometry_msgs::Pose2D &start, const geometry_msgs::Pose2D &goal,
                            double vmax, double wmax)
{
  estimate_msg.request.start = start;
  estimate_msg.request.goal = goal;
  estimate_msg.request.v_max = vmax;
  estimate_msg.request.w_max = wmax;
  estimate_srv.call(estimate_msg);
  return estimate_msg.response.seconds;
}

MoveResponse Monitor_IO::move(std::string robot_name, const geometry_msgs::Pose2D &goal, double vmax, double wmax)
{
  move_msg.request.robot_name = robot_name;
  move_msg.request.goal = goal;
  move_msg.request.v_max = vmax;
  move_msg.request.w_max = wmax;
  move_srv.call(move_msg);
  return move_msg.response;

}

StatusResponse Monitor_IO::status(std::string robot_name)
{
  status_msg.request.robot_name = robot_name;
  status_srv.call(status_msg);
  return status_msg.response;
}

