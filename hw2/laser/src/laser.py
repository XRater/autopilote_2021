#! /usr/bin/python3

import rospy
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan


class LaserTask:

	def __init__(self):
		self.rate = rospy.Rate(20)

		self.laser_subscriber = rospy.Subscriber('/base_scan', LaserScan, self.callback)
		self.marker_publisher = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

		self.threshold = 0.1

	def callback(self, laser: LaserScan):
		xs, ys = self.get_points(laser)
		self.rate.sleep()
		marker = self.create_marker(xs[is_ok], ys[is_ok])
		self.marker_publisher.publish(marker)

	def get_points(self, laser: LaserScan):
		laser_ranges = np.array(laser.ranges)
		laser_angles = np.arange(start=msg.angle_min, step=msg.angle_increment, end=msg.angle_max)

		diffs = np.abs(laser_ranges[:-1] - laser_ranges[1:])
		big_diffs = diffs > threshold
		outliers = np.logical_and(np.insert(big_diffs, 0, False), np.append(big_diffs, False))

		xs, ys = laser_ranges * np.cos(laser_angles), laser_ranges * np.sin(laser_angles)
		return xs[~outliers], ys[~outliers]

	def create_marker(self, xs, ys):
		marker = Marker()
		marker.type = marker.POINTS
		marker.points = [Point(x, y, 0.0) for x, y in zip(xs, ys)]

		marker.id = 0
		marker.action = 0
		marker.header.frame_id = "base_laser_link"

		marker.color.r = 1.
		marker.color.g = 0.
		marker.color.b = 0.
		marker.color.a = 1.

		marker.scale.x = 0.03
		marker.scale.y = 0.03
		marker.scale.z = 0.03

		return marker


rospy.init_node('viz_map')
laser_task = LaserTask()

rospy.spin()
