#! /usr/bin/python3

import rospy
from geometry_msgs.msg import Twist

rospy.init_node('turtle_move')

pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)
msg = Twist()
msg.linear.x = 1.0
msg.angular.z = 1.0

r = rospy.Rate(0.5)
while not rospy.is_shutdown():
	pub.publish(msg)
	r.sleep()
