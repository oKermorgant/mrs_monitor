#include <ros/ros.h>
#include <chrono>
#include <thread>
#include <rosgraph_msgs/Clock.h>

using namespace std::chrono;
using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sim_time");
  ros::NodeHandle nh;

  const auto speed = ros::NodeHandle("~").param("speed", 1.);

  auto clock_pub = nh.advertise<rosgraph_msgs::Clock>("/clock", 10);
  rosgraph_msgs::Clock clock;

  const auto t0 = steady_clock::now();
  const auto dt = 10ms;

  while(ros::ok())
  {
    clock.clock.fromNSec(speed*(steady_clock::now() - t0).count());
    clock_pub.publish(clock);
    std::this_thread::sleep_for(dt);
  }
}
