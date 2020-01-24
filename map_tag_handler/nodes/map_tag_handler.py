#!/usr/bin/env python

import os
import sys
import time
import traceback
import actionlib
import matplotlib.pyplot as plt
import numpy as np
import rospy
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Int16, Int16MultiArray
from explore_labyrinth_srv.srv import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# specify directions
FRONT_LEFT = 0
Front_RIGHT = 1
BACK_LEFT = 2
BACK_RIGHT = 3

PI = 3.1415926535897

if os.name == 'nt':
    pass
else:
    import termios

class MapTagHandler:

    def __init__(self):
        self._map_saver = rospy.ServiceProxy("map", OccupancyGrid)
        



def main():
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('map_tag_handler')
    try:
        mth = MapTagHandler()
        rospy.spin()
    except Exception as e:
        print e
        traceback.print_exc()


if __name__ == "__main__":
    main()
