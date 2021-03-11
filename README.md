# Helper library for multi-robot task allocation

This package offers a few services to help managing a multi-robot system (in 2D).

All planning / control are done through the nav stack. It could be anything, as long as it is possible to plan without moving, and plan / control a robot with feedback.

# Convention

* Robots have names and are assumed to be spawned in the corresponding namespace
* They use {name}/base_link and {name}/odom as main links
* They run `move_base` in their namespace

# Exposed services

The main node is `mrs_monitor` and exposes three services, that are all defined in the package:
* `estimate(start, goal, vmax, wmax)`: time-estimation of the trajectory between two 2D poses
* `status(name)`: get the status of a robot. It is either `-1.` if the robot is waiting, of the estimated time to arrival if it is doing something
* `move(name, goal, vmax, wmax)`: request a robot to move

# Demo node

The `demo.cpp` code shows how to have several robots planning and moving around. It also shows a helper library (`mrs_monitor_io`) to hide the low-level service calls.
