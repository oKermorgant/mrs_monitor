#include <mrs_monitor/robot.h>
#include <dynamic_reconfigure/Reconfigure.h>

using namespace mrs_monitor;

tf2_ros::Buffer* Robot::tf_buffer;
dynamic_reconfigure::Reconfigure Robot::max_vel_config;

Robot::Robot(ros::NodeHandle &nh, std::string name) : nh(nh), name(name), frame(name + "/base_link"), move_base_ns("/" + name + "/move_base/")
{
  goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/" + name + "/move_base_simple/goal", 10);
  track_pub = nh.advertise<nav_msgs::Path>("/" + name + "/move_base_simple/plan", 10);
  remaining_path.header.frame_id = "map";

  // get max v / w
  vmax_inv = 1./nh.param(move_base_ns + "DWAPlannerROS/max_vel_trans", 1.);
  wmax_inv = 1./nh.param(move_base_ns + "DWAPlannerROS/max_vel_rot", 2.);

  // allow changing these values
  max_vel_srv = nh.serviceClient<dynamic_reconfigure::Reconfigure>(move_base_ns + "DWAPlannerROS/set_parameters");

  // dummy last_goal
  last_goal.position.z = 100;
  last_goal.orientation.w = 1;
}

void Robot::updateMaxVel(double vmax, double wmax)
{
  max_vel_config.request.config.doubles.clear();
  writeParamUpdate(max_vel_config.request.config.doubles, "max_vel_trans", vmax, this->vmax_inv);
  writeParamUpdate(max_vel_config.request.config.doubles, "max_vel_rot", wmax, this->wmax_inv);
  if(max_vel_config.request.config.doubles.size())
    max_vel_srv.call(max_vel_config);
}

bool Robot::trackGoal(const geometry_msgs::Pose2D &pose, double vmax, double wmax)
{
  if(status == Status::MOVING)
    return false;

  updateMaxVel(vmax, wmax);
  convert(pose, last_goal);
  last_goal.position.z = 0;

  if(track_pub.getNumSubscribers())
  {
    track_pub.publish(remaining_path);
  }
  else
  {
    // request eg move_base to find a path + track it
    static geometry_msgs::PoseStamped goal = [](){geometry_msgs::PoseStamped goal;goal.header.frame_id = "map";return goal;}();
    goal.pose = last_goal;
    goal.header.stamp = ros::Time::now();
    goal_pub.publish(goal);
  }

  // monitor this goal
  status = Status::MOVING;
  status_sub = nh.subscribe<actionlib_msgs::GoalStatusArray>(move_base_ns + "status", 10,
                                                             &Robot::statusCallback, this);
  remaining_path_pub = nh.advertise<nav_msgs::Path>(move_base_ns + "plan", 10);

  return true;
}

double Robot::timeTo(const PoseStamped &pose)
{
  updateCurrentPose();
  return travelTime(cur_pose, pose, vmax_inv, wmax_inv);
}

double Robot::timeToGoal()
{
  if(status == Status::WAITING)
    return -1;

  double t(travelTime(remaining_path.poses, vmax_inv, wmax_inv));
  if(!remaining_path.poses.empty())
    t += timeTo(remaining_path.poses.front());
  return t;
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

void Robot::getPose(PoseStamped &pose) const
{
  convert(tf_buffer->lookupTransform("map", frame, ros::Time(0)).transform,
          pose);
}

void Robot::statusCallback(const actionlib_msgs::GoalStatusArrayConstPtr &goal_status)
{
  const bool moving = std::any_of(goal_status->status_list.begin(),
                                  goal_status->status_list.end(),
                                  [](const actionlib_msgs::GoalStatus &status)
  {return status.status == status.ACTIVE;});

  if(moving)
  {
    status = Status::MOVING;
  }
  else
  {
    if(status == Status::MOVING)
    {
      last_goal.position.z = 100;
      // stop listening to useless messages
      status = Status::WAITING;
      status_sub.shutdown();
    }
  }
  publishRemainingPath();
}

void Robot::publishRemainingPath()
{
  if(status == Status::WAITING)
    remaining_path.poses.clear();

  if(!remaining_path.poses.empty())
  {
    // remove elements from path
    updateCurrentPose();
    auto &poses(remaining_path.poses);
    size_t nearest(0);
    auto smallest(travelTime(cur_pose, poses.front(), vmax_inv, wmax_inv));
    for(size_t i = 1; i < poses.size(); ++i)
    {
      auto t(travelTime(cur_pose, poses[i], vmax_inv, wmax_inv));
      if(t < smallest)
      {
        t = smallest;
        nearest = i;
      }
    }
    poses.erase(poses.begin(), poses.begin()+nearest);
  }

  remaining_path.header.stamp = ros::Time::now();
  remaining_path_pub.publish(remaining_path);
}
