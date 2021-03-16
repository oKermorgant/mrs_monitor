// basic includes
#include <ros/ros.h>
#include <thread>

// to communicate easily
#include <mrs_monitor/monitor_io.h>

using namespace mrs_monitor;
using namespace std::chrono_literals;

inline double rand(double min, double max)
{
  return min + (max-min)*((double)rand()/RAND_MAX);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo_cpp");
  ros::NodeHandle nh;

  srand(time(nullptr));

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
    if(std::any_of(robots.begin(), robots.end(), [](const auto &robot)
    {return robot.seconds_to_goal < 0;}))
    {
      geometry_msgs::Pose2D goal;
      do {
        goal = pose2D(rand(-4.6, 10.6), rand(-6.4, 4.42), rand(-M_PI, M_PI));
      } while(!io.validGoal(goal));

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
