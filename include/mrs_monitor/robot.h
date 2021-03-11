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

  bool move(const Pose2D &pose, double vmax, double wmax);

  bool getPose(Pose2D &pose);

  Pose lastGoal() const
  {
    return last_goal;
  }

  double timeToGoal();

private:

  // for now these are loaded once and for all in constructor
  double vmax = 1;
  double wmax = 0.1;
  ros::ServiceClient max_vel_srv;
  static dynamic_reconfigure::Reconfigure max_vel_config;

  void updateMaxVel(double vmax, double wmax);

  geometry_msgs::Pose last_goal;
  ros::NodeHandle &nh;
  std::string name;
  std::string frame;
  std::string move_base_ns;
  Status status = Status::WAITING;

  static tf2_ros::Buffer *tf_buffer;


  ros::Subscriber remaining_path_sub, status_sub;
  ros::Publisher goal_pub;  
  std::vector<PoseStamped> remaining_path;

  void remainingPathCallback(const nav_msgs::PathConstPtr &path)
  {
    remaining_path = path->poses;
  }

  void statusCallback(const actionlib_msgs::GoalStatusArrayConstPtr &goal_status)
  {
    const bool moving = std::any_of(goal_status->status_list.begin(),
                              goal_status->status_list.end(),
                              [](const actionlib_msgs::GoalStatus &status)
    {return status.status == status.ACTIVE;});

    if(moving)
    {
      status = Status::MOVING;
    }
    else if(status == Status::MOVING)
    {
      last_goal.position.z = -100;
      // stop listening to useless messages
      status = Status::WAITING;
      remaining_path_sub.shutdown();
      status_sub.shutdown();
    }
  }

  template <class Doubles>
  static void writeParamUpdate(Doubles &doubles, std::string name, double new_val, double &cur_val)
  {
    if(new_val > 0 && new_val != cur_val)
    {
      cur_val = new_val;
      doubles.push_back({});
      doubles.back().name = name;
      doubles.back().value = new_val;
    }
  }
};

}

#endif // MRS_MONITOR_ROBOT_H
