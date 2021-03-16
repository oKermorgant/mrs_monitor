#ifndef MRS_MONITOR_IO_H
#define MRS_MONITOR_IO_H

// to communicate with mrs_monitor
#include <ros/ros.h>
#include <mrs_monitor/Estimate.h>
#include <mrs_monitor/Move.h>
#include <mrs_monitor/Status.h>

namespace mrs_monitor
{


// helper to generate a geometry_msgs::Pose2D
inline geometry_msgs::Pose2D pose2D(double x, double y, double theta)
{
  geometry_msgs::Pose2D pose;
  pose.x = x;
  pose.y = y;
  pose.theta = theta;
  return pose;
}

// basic robot IO

struct Robot_IO
{
  static ros::NodeHandle* nh;
  std::string name;
  geometry_msgs::Pose2D pose;
  double seconds_to_goal = -1;
  double vmax=0, wmax=0;
  inline Robot_IO(std::string name) : name(name)
  {
    // get vmax / wmax as params
    const auto move_base_ns("/" + name + "/move_base/");
    vmax = nh->param(move_base_ns + "DWAPlannerROS/max_vel_trans", 1.);
    wmax = nh->param(move_base_ns + "DWAPlannerROS/max_vel_rot", 2.);
  }

  inline void updateMaxVel(double &vmax, double &wmax) const
  {
    if(vmax == 0)
      vmax = this->vmax;
    if(wmax == 0)
      wmax = this->wmax;
  }

  inline void printStatus() const
  {
    std::cout << name << ": ";
    if(seconds_to_goal < 0)
      std::cout << "waiting instruction";
    else
      std::cout << seconds_to_goal << " seconds to goal ";
    std::cout << " @ (" << pose.x << ", " << pose.y << ", " << pose.theta << ")" << std::endl;
  }
};

// IO to call services
class Monitor_IO
{
public:
  Monitor_IO(ros::NodeHandle &nh, std::string ns = "mrs_monitor");

  bool exists(std::string name) const;

  // low-level calls
  double estimate(const geometry_msgs::Pose2D &start, const geometry_msgs::Pose2D &goal,
                                  double vmax=0, double wmax=0);
  MoveResponse move(std::string robot_name, const geometry_msgs::Pose2D &goal, double vmax=0, double wmax=0);
  StatusResponse status(std::string robot_name);

  // high-level
  inline double estimate(const Robot_IO &robot, const geometry_msgs::Pose2D &goal, double vmax=0, double wmax=0)
  {
    robot.updateMaxVel(vmax, wmax);
    return estimate(robot.pose, goal, vmax, wmax);
  }

  inline void move(const Robot_IO &robot, const geometry_msgs::Pose2D &goal, double vmax=0, double wmax=0)
  {
    robot.updateMaxVel(vmax, wmax);
    move(robot.name, goal, vmax, wmax);
  }

  inline void updateStatus(Robot_IO &robot)
  {
    auto resp(status(robot.name));
    robot.pose = resp.pose;
    robot.seconds_to_goal = resp.seconds_to_goal;
  }

  inline bool validGoal(const geometry_msgs::Pose2D &goal)
  {
    //static geometry_msgs::Pose2D start;
    return estimate(goal, goal, 1, 1) >= 0.;
  }

private:

  ros::NodeHandle &nh;
  ros::ServiceClient estimate_srv, move_srv, status_srv;
  Estimate estimate_msg;
  Move move_msg;
  Status status_msg;
};






}


#endif
