#include <mrs_monitor/robot.h>
#include <dynamic_reconfigure/Reconfigure.h>

using namespace mrs_monitor;

tf2_ros::Buffer* Robot::tf_buffer;
dynamic_reconfigure::Reconfigure Robot::max_vel_config;

Robot::Robot(ros::NodeHandle &nh, std::string name) : nh(nh), name(name), frame(name + "/base_link"), move_base_ns("/" + name + "/move_base/")
{
  goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/" + name + "/move_base_simple/goal", 10);

  // get max v / w
  vmax = nh.param(move_base_ns + "DWAPlannerROS/max_vel_trans", 1.);
  wmax = nh.param(move_base_ns + "DWAPlannerROS/max_vel_rot", 2.);

  // allow changing these values
  max_vel_srv = nh.serviceClient<dynamic_reconfigure::Reconfigure>(move_base_ns + "DWAPlannerROS/set_parameters");

  // dummy last_goal
  last_goal.position.z = 100;
  last_goal.orientation.w = 1;
}

void Robot::updateMaxVel(double vmax, double wmax)
{
  max_vel_config.request.config.doubles.clear();
  writeParamUpdate(max_vel_config.request.config.doubles, "max_vel_x", vmax, this->vmax);
  writeParamUpdate(max_vel_config.request.config.doubles, "max_vel_theta", wmax, this->wmax);
  if(max_vel_config.request.config.doubles.size())
    max_vel_srv.call(max_vel_config);
}

bool Robot::move(const geometry_msgs::Pose2D &pose, double vmax, double wmax)
{
  if(status == Status::MOVING)
    return false;

  updateMaxVel(vmax, wmax);

  static geometry_msgs::PoseStamped goal = [](){geometry_msgs::PoseStamped goal;goal.header.frame_id = "map";return goal;}();
  convert(pose, goal.pose);
  last_goal = goal.pose;
  goal.header.stamp = ros::Time::now();
  goal_pub.publish(goal);

  // monitor this goal
  status = Status::MOVING;
  remaining_path_sub = nh.subscribe<nav_msgs::Path>(move_base_ns + "NavfnROS/plan", 10,
                                                  &Robot::remainingPathCallback, this);
  status_sub = nh.subscribe<actionlib_msgs::GoalStatusArray>(move_base_ns + "status", 10,
                                                             &Robot::statusCallback, this);

  return true;
}

double Robot::timeToGoal()
{
  if(status == Status::WAITING)
    return -1;

  return travelTime(remaining_path, vmax, wmax);
}

bool Robot::getPose(geometry_msgs::Pose2D &pose)
{
  if(tf_buffer->canTransform("map", frame, ros::Time(0)))
  {
    convert(tf_buffer->lookupTransform("map", frame, ros::Time(0)).transform,
            pose);
    return true;
  }
  return false;
}
