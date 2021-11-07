#! /usr/bin/python3

import rospy
import numpy as np

from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point


class LaserTask:

    def __init__(self):
        self.rate = rospy.Rate(20)

        self.laser_subscriber = rospy.Subscriber('/base_scan', LaserScan, self.callback)
        self.marker_publisher = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        self.grid_publisher = rospy.Publisher('/map_topic', OccupancyGrid, queue_size=10)

        self.threshold = 0.05
        self.tile_size = 0.1
        self.map_size = 200
        
        self.step = 0

    def callback(self, laser: LaserScan):
        xs, ys = self.get_points(laser)

        self.rate.sleep()

        marker = self.create_marker(xs, ys)
        self.marker_publisher.publish(marker)
        grid = self.create_grid(xs, ys)
        self.grid_publisher.publish(grid)
        
        self.step += 1

    def get_points(self, laser: LaserScan):
        laser_ranges = np.array(laser.ranges)
        laser_angles = np.arange(start=laser.angle_min, step=laser.angle_increment, stop=laser.angle_max)

        diffs = np.abs(laser_ranges[:-1] - laser_ranges[1:])
        big_diffs = diffs > self.threshold
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

        marker.color.r = 0.
        marker.color.g = 0.
        marker.color.b = 1.
        marker.color.a = 1.

        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        return marker

    def create_grid(self, xs, ys):
        grid = OccupancyGrid()
        grid.header.frame_id = 'base_laser_link'
        grid.header.seq = self.step

        grid.info.width = self.map_size
        grid.info.height = self.map_size
        grid.info.resolution = self.tile_size

        points_in_cell = np.zeros((grid.info.width, grid.info.height), dtype=int)

        half_size = self.map_size * self.tile_size / 2
        for x, y in zip(xs, ys):
            if abs(x) < half_size and abs(y) < half_size:
                i = int((x + half_size) // self.tile_size)
                j = int((y + half_size) // self.tile_size)
                points_in_cell[i][j] += 1
        
        # may be done smarter
        grid_data = (100 * (1 - 1 / (points_in_cell * 5 + 1))).astype(np.int)

        grid.info.origin.position.x = -half_size
        grid.info.origin.position.y = -half_size

        grid.data = list(grid_data.transpose(1, 0).reshape(-1))
        return grid

rospy.init_node('laser')
laser_task = LaserTask()

rospy.spin()
