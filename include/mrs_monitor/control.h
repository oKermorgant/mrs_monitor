#ifndef MRS_MONITOR_CONTROL_H
#define MRS_MONITOR_CONTROL_H

#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/transform_listener.h>
#include <dwa_local_planner/dwa_planner_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <nav_msgs/GetPlan.h>

namespace mrs_monitor
{

namespace
{

inline void convert(const geometry_msgs::TransformStamped &tf, geometry_msgs::PoseStamped &pose)
{
  pose.header.frame_id = tf.header.frame_id;
  pose.pose.position.x = tf.transform.translation.x;
  pose.pose.position.y = tf.transform.translation.y;
  pose.pose.position.z = tf.transform.translation.z;
  pose.pose.orientation.w = tf.transform.rotation.w;
  pose.pose.orientation.x = tf.transform.rotation.x;
  pose.pose.orientation.y = tf.transform.rotation.y;
  pose.pose.orientation.z = tf.transform.rotation.z;
}

}

class Controller
{
public:
  Controller();

  template <class LocalPlanner>
  void initLocalPlanner()
  {
    // init costmap after potentially changing params
    costmap = std::make_unique<costmap_2d::Costmap2DROS>("local_costmap", buffer);

    // init the local planner
    local_planner = std::make_unique<LocalPlanner>();
    local_planner->initialize("DWAPlannerROS", &buffer, costmap.get());
  }

  template <class T>
  inline void setLocalParam(std::string name, T value)
  {
    priv.setParam(name, value);
  }

private:

  enum class Status {MOVING, DONE};

  ros::NodeHandle nh, priv;
  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener tl;
  std::unique_ptr<costmap_2d::Costmap2DROS> costmap;

  ros::Publisher cmd_vel_pub;
  geometry_msgs::Twist cmd_vel;
  std::unique_ptr<nav_core::BaseLocalPlanner> local_planner;
  Status status = Status::DONE;

  // emulate planning from topic
  ros::Subscriber goal_sub;
  ros::ServiceClient plan_srv;
  inline void goalCallback(const geometry_msgs::PoseStampedConstPtr &goal);

  // also track an already planned path
  ros::Subscriber plan_sub;
  inline void beginTrackingPath(const nav_msgs::Path &path)
  {
    if(local_planner->setPlan(path.poses))
    {
      status = Status::MOVING;
      timer.setPeriod(refresh_moving);
    }
  }

  // emulate move_base goal feedback
  ros::Publisher status_pub;
  void refresh();
  ros::Timer timer;
  ros::Duration refresh_still, refresh_moving;
};

}

#endif
