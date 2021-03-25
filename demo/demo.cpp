// basic includes
#include <ros/ros.h>
#include <thread>

// to communicate easily
#include <mrs_monitor/monitor_io.h>
#include <nav_msgs/GetMap.h>

using namespace mrs_monitor;
using namespace std::chrono_literals;

struct RandCoord
{
  inline double rand(double min, double max) const
  {
    return min + (max-min)*((double)std::rand()/RAND_MAX);
  }

  RandCoord()
  {
    // get map dimensions
    ros::NodeHandle nh;
    ros::ServiceClient map(nh.serviceClient<nav_msgs::GetMap>("/static_map"));

    nav_msgs::GetMap full_info;
    map.call(full_info);
    const auto &info(full_info.response.map.info);
    const double x0(info.origin.position.x);
    const double y0(info.origin.position.y);
    const double width(info.width*info.resolution);
    const double height(info.height*info.resolution);

    const double margin(0.3);

    xm = x0+margin;
    xM = x0+width-margin;
    ym = y0+margin;
    yM = y0+height-margin;

    std::cout << "Will generate " << xm << " <= x <= " << xM << " / " << ym << " <= y <= " << yM << std::endl;
    srand(time(nullptr));
  }

  geometry_msgs::Pose2D operator()() const
  {
    static geometry_msgs::Pose2D pose;
    pose.x = rand(xm,xM);
    pose.y = rand(ym, yM);
    pose.theta = rand(-M_PI, M_PI);
    return pose;
  }
  double xm, xM, ym, yM;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo_cpp");
  ros::NodeHandle nh;

  RandCoord rand_coord;

  Monitor_IO io(nh);

  // robot basic interface
  std::vector<Robot_IO> robots;
  for(int i = 1; i <= 10; ++i)
  {
    std::string name("robot" + std::to_string(i));
    if(io.exists(name))
      robots.emplace_back(name);
  }
  std::cout << "Found " << robots.size() << " robots" << std::endl;

  const auto min_avail_robot = robots.size() < 4 ? 1 : sqrt(robots.size());

  ros::Rate rate(0.2);

  while(ros::ok())
  {
    std::cout << std::endl;
    // print status of all robots
    // negative seconds_to_goal means not working for now
    for(auto &robot: robots)
    {
      io.updateStatus(robot);
      robot.printStatus();
    }

    // if any available, generate a random goal
    if(std::count_if(robots.begin(), robots.end(), [](const auto &robot)
    {return robot.seconds_to_goal < 0;}) >= min_avail_robot)
    {
      geometry_msgs::Pose2D goal;
      do {
        goal = rand_coord();
      } while(io.estimate(robots[0], goal) < 0);

      std::cout << "New goal @ " << goal.x << ", " << goal.y << std::endl;

      // find available robot closest to goal
      double smallest_time(-1);
      Robot_IO* closest(nullptr);
      for(auto &robot: robots)
      {
        io.updateStatus(robot);
        if(robot.seconds_to_goal < 0)
        {
          auto estimate = io.estimate(robot, goal);
          std::this_thread::sleep_for(20ms);
          if(estimate < smallest_time || smallest_time < 0)
          {
            smallest_time = estimate;
            closest = &robot;
          }
        }
      }
      if(closest)
      {
        // ask this robot to move
        io.move(*closest, goal);
      }
    }
    else
      std::cout << "All robots are currently moving" << std::endl;

    rate.sleep();
  }

}
