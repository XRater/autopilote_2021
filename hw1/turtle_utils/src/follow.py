#! /usr/bin/python3

import rospy
import math
import time
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class FollowTask:

	def __init__(self, actor_name='actor', target_name='turtle1'):
		self.actor_name = actor_name
		self.target_name = target_name
		
		self.target_position = None
		self.actor_position = None
		rospy.Subscriber(
			f'/{self.target_name}/pose', Pose, 
			self.update_target_position
		)
		rospy.Subscriber(
			f'/{self.actor_name}/pose', Pose,
			 self.update_actor_position
		)
		
		self.actor_commander = rospy.Publisher('/{self.actor_name}/cmd_vel', Twist, queue_size=1)

	def update_target_position(self, msg):
		self.target_position = msg

	def update_actor_position(self, msg):
		self.actor_position = msg
		
	def send_follow_command(self):
		if self.actor_position is None or self.target_position is None:
			return
		position = self.target_position
		distance = (self.actor_position.x - position.x) ** 2 + (self.actor_position.y - position.y) ** 2
		distance = math.sqrt(distance)
		if distance < 0.05:
			return
		target_angle = math.acos((position.y - self.actor_position.y) / distance)
		if self.actor_position.x > position.x:
			target_angle = math.pi * 2 - target_angle
		target_angle = -target_angle + math.pi / 2
		rot =target_angle - self.actor_position.theta
		if rot < -math.pi:
			rot += 2 * math.pi
		if rot > math.pi:
			rot = math.pi * 2 - rot
		msg = Twist()
		msg.linear.x = min(1, distance)
		msg.angular.z = rot
		self.actor_commander.publish(msg)

rospy.init_node('turtle_follow')
follow_task = FollowTask()

r = rospy.Rate(0.9)
while not rospy.is_shutdown():
	follow_task.send_follow_command()
	r.sleep() 
