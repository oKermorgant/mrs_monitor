#ifndef MRS_MONITOR_FLEET_H
#define MRS_MONITOR_FLEET_H

#include <mrs_monitor/robot.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <mrs_monitor/Estimate.h>
#include <mrs_monitor/Move.h>
#include <mrs_monitor/Status.h>

namespace mrs_monitor
{

class Fleet
{
public:
  Fleet();

private:

  Robot* getRobot(const std::string &name);

  ros::NodeHandle nh;
  std::vector<std::unique_ptr<Robot>> fleet;
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;
  ros::Publisher goals_pub;

  // advertized services and their callbacks
  ros::ServiceServer estimate_srv, move_srv, status_srv;
  bool estimateCallback(EstimateRequest &req, EstimateResponse &res);
  bool moveCallback(MoveRequest &req, MoveResponse &);
  bool statusCallback(StatusRequest &req, StatusResponse &res);  

  // global planning service - used for all estimations
  ros::ServiceClient plan_srv;

};


}

#endif // MRS_MONITOR_FLEET_H
