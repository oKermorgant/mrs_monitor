#!/usr/bin/env python

import rospy
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped

rospy.init_node('test_plan')

make_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)

start = PoseStamped()
goal = PoseStamped()

start.header.frame_id = goal.header.frame_id = 'map'

start.pose.orientation.w = 1
goal.pose.orientation.w = 1

tol = 0.05


goal.pose.position.x = 10
goal.pose.position.y = 2

t0 = rospy.Time.now().to_sec()

start.header.stamp =  rospy.Time.from_sec(t0)
goal.header.stamp = rospy.Time.from_sec(t0+20)

path = make_plan(start, goal, tol).plan.poses

print('Got plan of length {}'.format(len(path)))

if len(path):
    for name, pose in (('start', path[0]), ('end', path[-1])):
        print('{} @ {}:\n {} '.format(name, pose.header.stamp, pose.pose))
        
    print('Duration: {}'.format(path[-1].header.stamp - path[0].header.stamp))
    
    
