#!/usr/bin/env python

import os
import sys
import time
import traceback
import actionlib
import rospy
import numpy as np
from std_msgs.msg import Int16
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry

from sensor_msgs.msg import LaserScan

if os.name == 'nt':
    pass
else:
    import termios

SLEEP_TIME = 3
MIN_DISTANCE = 0.20
# specify directions
FRONT_LEFT = 0
FRONT_RIGHT = 1
BACK_LEFT = 2
BACK_RIGHT = 3

class AvoidObstacless:

    def __init__(self):
        self._lidar_data = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self._ranges = None
        while self._ranges is None:
            time.sleep(2)
        self._ninety_degree_index = len(self._ranges)/4
        self._oneigthy_degree_index = len(self._ranges)/2
        self._twoseventy_degree_index = self._ninety_degree_index + self._oneigthy_degree_index
        self._threesixty_degree_index = len(self._ranges)
        self._pub = rospy.Publisher('/obstacle_avoidance', Int16, queue_size=10)


    def control_loop(self):
        while not rospy.is_shutdown():
            # evaluate ranges
            r = self._ranges
            indi = self.get_indicess_of_obstacle(r)
            if len(indi) != 0:
                dir_obstacle = self.get_direction(indi[0])
                dir_max_space = self.get_direction_unwedge(dir_obstacle)
            else:
                dir_max_space = -1
            print dir_max_space
            self.publish(dir_max_space)
            time.sleep(SLEEP_TIME)

    def publish(self, direction_to_go):
        self._pub.publish(Int16(direction_to_go))

    def get_direction(self, index):
        if 0 <= index < self._ninety_degree_index:
            return FRONT_LEFT
        if self._ninety_degree_index <= index < self._oneigthy_degree_index:
            return FRONT_RIGHT
        if self._oneigthy_degree_index <= index < self._twoseventy_degree_index:
            return BACK_LEFT
        if self._twoseventy_degree_index <= index < self._threesixty_degree_index:
            return BACK_RIGHT

    def get_direction_unwedge(self, index):
        obs = self.get_direction(index)
        if obs is FRONT_LEFT:
            return BACK_RIGHT
        if obs is FRONT_RIGHT:
            return BACK_LEFT
        if obs is BACK_RIGHT:
            return FRONT_LEFT
        if obs is BACK_LEFT:
            return FRONT_RIGHT

    def get_indicess_of_obstacle(self, ranges):
        i = 0
        j = 0
        indicess = {}
        while i < len(ranges):
            # get rid of -inf values
            if 0 <= ranges[i] < MIN_DISTANCE:
                indicess[j] = i
                j = j +1
            i = i + 1
        return indicess

    def lidar_callback(self, data):
        self._ranges = data.ranges


def main():
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('movement_controler')
    try:
        mc = AvoidObstacless()
        mc.control_loop()
    except Exception as e:
        print e
        traceback.print_exc()


if __name__ == "__main__":
    main()
