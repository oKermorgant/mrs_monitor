#ifndef MRS_MONITOR_ROBOT_H
#define MRS_MONITOR_ROBOT_H

#include <tf2_ros/buffer.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <mrs_monitor/geometry.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <dynamic_reconfigure/Reconfigure.h>

namespace mrs_monitor
{

class Robot
{

public:

  enum class Status{WAITING, MOVING};

  Robot(ros::NodeHandle &nh, std::string name);

  bool is(const std::string &name) const
  {
    return this->name == name;
  }

  static void listenThrough(tf2_ros::Buffer &buffer)
  {
    tf_buffer = &buffer;
  }

  bool trackGoal(const Pose2D &pose, double vmax, double wmax);

  bool getPose(Pose2D &pose);
  void getPose(PoseStamped &pose) const;
  inline void updateCurrentPose()
  {
    getPose(cur_pose);
  }

  double timeTo(const PoseStamped &pose);

  inline void setPath(const nav_msgs::Path &path)
  {
    remaining_path = path;
  }

  inline Pose lastGoal() const
  {
    return last_goal;
  }

  double timeToGoal();

private:

  // for now these are loaded once and for all in constructor
  double vmax_inv = 1;
  double wmax_inv = 0.1;
  ros::ServiceClient max_vel_srv;
  static dynamic_reconfigure::Reconfigure max_vel_config;

  void updateMaxVel(double vmax_inv, double wmax_inv);
  void publishRemainingPath();

  geometry_msgs::Pose last_goal;
  geometry_msgs::PoseStamped cur_pose;
  ros::NodeHandle &nh;
  std::string name;
  std::string frame;
  std::string move_base_ns;
  Status status = Status::WAITING;

  static tf2_ros::Buffer *tf_buffer;

  ros::Subscriber status_sub;
  ros::Publisher goal_pub, remaining_path_pub, track_pub;
  nav_msgs::Path remaining_path;

  void statusCallback(const actionlib_msgs::GoalStatusArrayConstPtr &goal_status);

  template <class Doubles>
  static void writeParamUpdate(Doubles &doubles, std::string name, double vel_max, double &vel_max_inv)
  {
    if(vel_max > 0 && 1./vel_max != vel_max_inv)
    {
      vel_max_inv = 1./vel_max;
      doubles.push_back({});
      doubles.back().name = name;
      doubles.back().value = vel_max;
    }
  }
};

}

#endif // MRS_MONITOR_ROBOT_H
