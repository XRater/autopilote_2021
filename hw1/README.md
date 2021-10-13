roscore
rosrun turtlesim turtlesim_node
rosservice call /spawn "{x: 1.0, y: 1.0, theta: -1., name: 'actor'}"
rosrun turtle_utils follow.py
