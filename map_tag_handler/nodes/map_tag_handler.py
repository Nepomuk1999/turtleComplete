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
from map_tag_handler_srv.srv import *
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
        self._provide_tag_service = rospy.Service('/get_next_Tag', TagService, self.provide_next_tag)
        self._save_tags_service = rospy.Service('/save_tags', TagService, self.save_tags)
        occupancy_grid = rospy.wait_for_message("/map", OccupancyGrid)
        meta_data = occupancy_grid.info
        self._offset_x = meta_data.origin.position.x
        self._offset_y = meta_data.origin.position.y
        self._resolution = meta_data.resolution

        self._active_tags = None
        self._my_fount_tags_x = None
        self._my_fount_tags_y = None

        self._colaborator_fount_tags_x = None
        self._colaborator_fount_tags_x = None


        self.read_tags_from_file()
        self._distance_values = None
        self.calculate_distances()



    def provide_next_tag(self, msg):
        i = 0
        while i < len(self._active_tags):
            if self._active_tags[i] is True:
                self._active_tags[i] = False
                return TagServiceResponse(self.transform_to_meter(self._my_fount_tags_x[i], self._my_fount_tags_y[i]))

    def save_tags(self, data):
        self._my_fount_tags_x = data.data.x
        self._my_fount_tags_y = data.data.y
        self.write_to_file(self._my_fount_tags_x, self._my_fount_tags_y)

    def transform_to_pos(self, m_x, m_y):
        pos_x = np.int((m_x - self._offset_x) / self._resolution)
        pos_y = np.int((m_y - self._offset_y) / self._resolution)
        return pos_x, pos_y

    def transform_to_meter(self, pos_x, pos_y):
        m_x = pos_x * self._resolution + self._offset_x
        m_y = pos_y * self._resolution + self._offset_y
        return m_x, m_y

    def write_to_file(self, x_data, y_data):
        i = 0
        filenamex = "x.txt"
        filenamey = "y.txt"
        xfile = open(filenamex, 'w')
        yfile = open(filenamey, 'w')
        while i < len(x_data):
            xfile.write(x_data[i] + '\n')
            yfile.write(y_data[i] + '\n')
        xfile.close()
        yfile.close()

    def read_tags_from_file(self):
        i = 0
        self._my_fount_tags_x = []
        self._my_found_tags_y = []
        filenamex = "x.txt"
        filenamey = "y.txt"
        xfile = open(filenamex, 'r')
        yfile = open(filenamey, 'r')
        strx = xfile.readline()
        stry = yfile.readline()
        while len(strx) is not 0:
            self._my_fount_tags_x.append(int(strx))
            self._my_fount_tags_y.append(int(stry))
            strx = xfile.readline()
            stry = yfile.readline()
        xfile.close()
        yfile.close()

# check start vel
    def calculate_distances(self):
        print 'Calculating distances'
        occupancy_grid = rospy.wait_for_message("/map", OccupancyGrid)
        meta_data = occupancy_grid.info
        occupancy_map = occupancy_grid.data
        trimmed_map = np.array(occupancy_map)
        map_height = meta_data.height
        map_width = meta_data.width
        current_map = trimmed_map.reshape((map_width, map_height))
        # distances from each tag to each others
        self._distance_values = np.array(np.shape(len(self._my_fount_tags_x), len(self._my_fount_tags_x)))
        # index of start values
        for i in range(0, len(self._my_fount_tags_x)):
            print 'Calculation in progress Tagnr: ', i
            start_pose = np.array([self._my_fount_tags_x[i], self._my_fount_tags_x[i]])
            path = [start_pose]
            closed_list = []
            distance = 0
            distance_map = np.ndarray(np.shape(map_width, map_height))
            while len(path) > 0:
                current_path = path.pop(0)
                closed_list.append(current_path)
                current_x = current_path[0]
                current_y = current_path[1]
                # is wall
                if current_map[current_y, current_x] == 1:
                    continue
                if current_map[current_y, current_x] == 0:
                    distance_map[current_y][current_x] = distance
                directions = np.array([[current_x - 1, current_y], [current_x + 1, current_y], [current_x, current_y - 1], [current_x, current_y + 1]])
                np.random.shuffle(directions)
                for x in directions:
                    if not self.cointains_pos(x, closed_list):
                        if not self.cointains_pos(x, path):
                            path.append(x)
                distance += 1
            for j in range(0, len(self._my_fount_tags_x)):
                p1 = distance_map[self._my_fount_tags_x[i]][self._my_fount_tags_y[i]]
                p2 = distance_map[self._my_fount_tags_x[j]][self._my_fount_tags_y[j]]
                dist = abs(p1 - p2)
                self._distance_values[i][j] = dist
                print 'current calculated distances:'
                print self._distance_values
        print 'calculation done'

    def cointains_pos(self, array, array_array):
        for i in array_array:
            if i[0] == array[0]:
                if i[1] == array[1]:
                    return True
        return False


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
