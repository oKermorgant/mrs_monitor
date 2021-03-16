#include <mrs_monitor/planner.h>
#include <tf2_ros/static_transform_broadcaster.h>

using namespace mrs_monitor;

Planner::Planner(tf2_ros::Buffer &buffer) : costmap("global_costmap", buffer)
{
  planner.initialize("GlobalPlanner", &costmap);
}
/*
void waitForCostmap()
{
  // publish some TF
  geometry_msgs::TransformStamped map_tf;
  map_tf.child_frame_id = "global_costmap";
  map_tf.header.frame_id = "map";
  map_tf.transform.rotation.w = 1;
  tf2_ros::TransformBroadcaster br;

  ros::Rate rate(1);
  for(int i = 0; i < 5; ++i)
  //while(ros::ok())
  {

    std::cout << "Sending t = " << map_tf.header.stamp << std::endl;
    rate.sleep();
    ros::spinOnce();
  }
}*/

int main(int argc, char** argv)
{
  ros::init(argc, argv, "global_planner");

  ros::NodeHandle costmap_nh("~global_costmap");
  costmap_nh.setParam("robot_base_frame", "global_costmap");
  costmap_nh.setParam("robot_radius", 1.2*costmap_nh.param("robot_radius", .3));

  tf2_ros::Buffer buffer(ros::Duration(10));
  tf2_ros::TransformListener tl(buffer);

 /* geometry_msgs::TransformStamped map_tf;
  map_tf.child_frame_id = "global_costmap";
  map_tf.header.frame_id = "map";
  map_tf.transform.rotation.w = 1;
  tf2_ros::StaticTransformBroadcaster br;
  map_tf.header.stamp = ros::Time::now();
  br.sendTransform(map_tf);*/

  Planner planner(buffer);

  ros::spin();

}
