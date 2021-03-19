#ifndef GREEDY_CONTROL_H
#define GREEDY_CONTROL_H

#include <nav_core/base_local_planner.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/goal_functions.h>
#include <tf2/utils.h>

namespace mrs_monitor
{

class GreedyNoWalls : public nav_core::BaseLocalPlanner
{
public:

    // mandatory interface from BaseLocalPlanner
    GreedyNoWalls() : odom("odom") {}

    void initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros) override
    {
        this->tf = tf;
        this->costmap = costmap_ros;

        // load private parameters
        ros::NodeHandle priv("~" + name);

        v_gain = priv.param("v_gain", 4.);
        w_gain = priv.param("w_gain", 3.);
        xy_tolerance = priv.param("xy_goal_tolerance", 0.02);
        yaw_tolerance = priv.param("yaw_goal_tolerance", 0.1);
        v_max = priv.param("max_vel_trans", 0.55);
        w_max = priv.param("max_vel_rot", 3.);

        local_plan_pub = priv.advertise<nav_msgs::Path>("global_plan", 10);
        traj_pub = priv.advertise<nav_msgs::Path>("local_plan", 10);

        local_pose.header.frame_id = costmap->getBaseFrameID();
        local_pose.pose.orientation.w = 1;
    }

    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override
    {
        if(!updateLocalPlan())
        {
            // stop
            cmd_vel.linear.x = cmd_vel.linear.y = cmd_vel.angular.z = 0;
            return true;
        }

        // get last pose of local_plan
        const auto &goal(local_plan.back().pose);
        local_pose.header.stamp = ros::Time::now();


        base_local_planner::publishPlan({local_pose, local_plan.back()}, traj_pub);

        auto dx(goal.position.x);
        auto dy(goal.position.y);
        double dtheta = 0;

        if(sqrt(dx*dx + dy*dy) < xy_tolerance)
        {
            // almost there, just align
            dtheta = 2*atan2(goal.orientation.z, goal.orientation.w);

            if(std::abs(dtheta) < yaw_tolerance)
                goal_reached = true;
        }
        else
        {
            dtheta = atan2(dy, dx);
        }

        cmd_vel.linear.x = std::clamp(v_gain*dx, -v_max, v_max);
        cmd_vel.linear.y = std::clamp(v_gain*dy, -v_max, v_max);
        cmd_vel.angular.z = std::clamp(w_gain*dtheta, -w_max, w_max);
        return true;
    }
    inline bool isGoalReached() override
    {
        return goal_reached;
    }
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override
    {
        if(plan.size())
        {
            goal_reached = false;
            global_plan = plan;
            return true;
        }
        return false;
    }

private:

    inline const costmap_2d::Costmap2D & rawCostmap() const
    {
        return *(costmap->getCostmap());
    }

    // config
    double v_gain, w_gain, xy_tolerance, yaw_tolerance, v_max, w_max;

    // global plan
    bool goal_reached;
    geometry_msgs::PoseStamped local_pose;
    std::vector<geometry_msgs::PoseStamped> global_plan, local_plan;
    bool updateLocalPlan()
    {
        if(global_plan.empty())
        {
            local_plan.clear();
            return false;
        }
        costmap->getRobotPose(global_pose);
        //get the global plan in our frame
        if(!base_local_planner::transformGlobalPlan(
                    *tf,
                    global_plan,
                    global_pose,
                    *(costmap->getCostmap()),
                    costmap->getBaseFrameID(),
                    local_plan)) {
            ROS_WARN("Could not transform the global plan to the frame of the controller");
            return false;
        }

        //now we'll prune the plan based on the position of the robot
        //base_local_planner::prunePlan(global_pose, local_plan, global_plan);

        base_local_planner::publishPlan(local_plan, local_plan_pub);

        return !local_plan.empty();
    }
    ros::Publisher local_plan_pub, traj_pub;

    // geometry
    tf2_ros::Buffer *tf;
    costmap_2d::Costmap2DROS* costmap;
    geometry_msgs::PoseStamped global_pose;
    base_local_planner::OdometryHelperRos odom;
    nav_msgs::Odometry base_odom;
};
}



#endif // GREEDY_CONTROL_H
