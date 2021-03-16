#ifndef MRS_MONITOR_GEOMETRY_H
#define MRS_MONITOR_GEOMETRY_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/PoseStamped.h>

namespace mrs_monitor
{

using geometry_msgs::PoseStamped;
using geometry_msgs::Pose2D;
using geometry_msgs::Pose;
using geometry_msgs::Transform;

inline void convert(const Pose2D &pose2D, Transform &pose)
{
  pose.translation.x = pose2D.x;
  pose.translation.y = pose2D.y;
  pose.rotation.w = cos(pose2D.theta/2);
  pose.rotation.z = sin(pose2D.theta/2);
}

inline void convert(const Pose2D &pose2D, Pose &pose)
{
  pose.position.x = pose2D.x;
  pose.position.y = pose2D.y;
  pose.orientation.w = cos(pose2D.theta/2);
  pose.orientation.z = sin(pose2D.theta/2);
}

inline void convert(const Transform &tf, Pose2D &pose2D)
{
  pose2D.x = tf.translation.x;
  pose2D.y = tf.translation.y;
  pose2D.theta = 2*atan2(tf.rotation.z, tf.rotation.w);
}

inline void convert(const Transform &tf, PoseStamped &pose)
{
  pose.pose.position.x = tf.translation.x;
  pose.pose.position.y = tf.translation.y;
  pose.pose.orientation.w = tf.rotation.w;
  pose.pose.orientation.z = tf.rotation.z;
}

double travelTime(const PoseStamped &prev, const PoseStamped &next, double vmax_inv, double wmax_inv);
double travelTime(const std::vector<PoseStamped> &path, double vmax_inv, double wmax_inv);

}

#endif // MRS_MONITOR_GEOMETRY_H
