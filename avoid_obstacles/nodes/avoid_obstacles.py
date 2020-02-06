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

SLEEP_TIME = 2
MIN_DISTANCE = 0.17
DIST_MAX = 0.75
# specify directions
FRONT_LEFT = 0
FRONT_RIGHT = 3
BACK_LEFT = 2
BACK_RIGHT = 1

class AvoidObstacless:

    def __init__(self):
        self._lidar_data = rospy.Subscriber('scan', LaserScan, self.lidar_callback)
        self._ranges = None
        while self._ranges is None:
            time.sleep(2)
        self._ninety_degree_index = len(self._ranges)/4
        self._oneigthy_degree_index = len(self._ranges)/2
        self._twoseventy_degree_index = self._ninety_degree_index + self._oneigthy_degree_index
        self._threesixty_degree_index = len(self._ranges)
        self._pub = rospy.Publisher('obstacle_avoidance', Int16, queue_size=10)
        print '90', self._ninety_degree_index
        print '180', self._oneigthy_degree_index
        print '270', self._twoseventy_degree_index
        print '360', self._threesixty_degree_index


    def control_loop(self):
        while not rospy.is_shutdown():
            # evaluate ranges
            r = self._ranges
            # set max vesls to 0.3
            r = np.asarray(r)
            r[r > DIST_MAX] = DIST_MAX
            # get max vel of
            fr, br, bl, fl = self.sum_of_quaters(r)
            indi = self.get_indicess_of_obstacle(r)
            print 'indi of obst', indi
            if len(indi) != 0:
                dir_max_space = self.eval_dir_to_go(fr, br, bl, fl)
            else:
                dir_max_space = -1
            print 'dir to go', dir_max_space
            self.publish(dir_max_space)
            time.sleep(SLEEP_TIME)

    def sum_of_quaters(self, ranges):
        fl = 0.0
        fr = 0.0
        bl = 0.0
        br = 0.0
        for i in range(0, self._ninety_degree_index):
            fl = fl + ranges[i]
        for i in range(self._ninety_degree_index, self._oneigthy_degree_index):
            bl = bl + ranges[i]
        for i in range(self._oneigthy_degree_index, self._twoseventy_degree_index):
            br = br + ranges[i]
        for i in range(self._twoseventy_degree_index, self._threesixty_degree_index):
            fr = fr + ranges[i]
        return fr, br, bl, fl

    def eval_dir_to_go(self, fr, br, bl, fl):
        rmin = min(fr, br, bl, fl)
        if rmin is fr or rmin is fl:
            rmax = max(br, bl)
            if rmax is bl:
                return BACK_RIGHT
            if rmax is br:
                return BACK_LEFT
        elif rmin is br or rmin is bl:
            rmax = max(fr, fl)
            if rmax is fr:
                return FRONT_RIGHT
            if rmax is fl:
                return FRONT_LEFT

    def publish(self, direction_to_go):
        self._pub.publish(Int16(direction_to_go))

    def get_indicess_of_obstacle(self, ranges):
        i = 0
        j = 0
        indices = {}
        while i < len(ranges):
            # get rid of -inf values
            if ranges[i] > 0:
                if ranges[i] <= MIN_DISTANCE:
                    indices[j] = i
                    j = j + 1
            i = i + 1
        return indices

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
