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
from save_tag_msg.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point

# specify directions
FRONT_LEFT = 0
Front_RIGHT = 1
BACK_LEFT = 2
BACK_RIGHT = 3

PI = 3.1415926535897

STAT_SEARCH = 'search_tokens'
STAT_DRIVE = 'drive_to_tokens'

TAG_STAT_OPEN = 1
TAG_STAT_SER = 2
TAG_STAT_FOUND = 3

FIRST_TAG_TOLERANCE = 0.4

# TODO implement tolerance for tag matching
COMUNICATION_TOLERANCE = 0.4

if os.name == 'nt':
    pass
else:
    import termios

class MapTagHandler:

    def __init__(self):
        self.call_counter = 0
        self._provide_tag_service = rospy.Service('get_next_Tag', TagService, self.provide_next_tag)
        self._save_tags_service = rospy.Subscriber('save_tags_to_file', SaveTag, self.save_tags_callback)
        occupancy_grid = rospy.wait_for_message("map", OccupancyGrid)
        meta_data = occupancy_grid.info
        self._offset_x = meta_data.origin.position.x
        self._offset_y = meta_data.origin.position.y
        self._resolution = meta_data.resolution

        top_ser = rospy.get_param('topic_searching')
        top_rea = rospy.get_param('topic_reached')
        self._search_pub = rospy.Publisher(top_ser, Point, queue_size=10)
        self._search_sub = rospy.Subscriber(top_ser, Point, self.top_ser_callback())
        self._reached_pub = rospy.Publisher(top_rea, Point, queue_size=10)
        self._reached_sub = rospy.Subscriber(top_rea, Point, self.top_rea_callback)

        self._start_x_coord = 0.0
        self._start_y_coord = 0.0

        self._active_tags = []
        self._my_found_tags_x = []
        self._my_found_tags_y = []

        self._last_send_tag_index = -1

        self._distance_values = None
        self._stat = STAT_SEARCH
        if self.stat == STAT_DRIVE:
            self.read_tags_from_file()
            self.calculate_distances()
        print 'end init'

    def top_ser_callback(self, data):
        pass

    def top_ser_pub(self, x, y):
        pass

    def top_rea_callback(self, data):
        pass

    def top_rea_pub(self, x,y):
        pass

    def provide_next_tag(self, msg):
        if self.call_counter == 0:
            print 'first call'
            print 'start_x:', msg.current_pose_x
            print 'start_y:', msg.current_pose_y
            self._start_x_coord, self._start_y_coord = self.transform_to_pos(msg.current_pose_x, msg.current_pose_y)
            goal_x, goal_y = self.find_first_tag(self._start_x_coord, self._start_y_coord)
            goal_x, goal_y = self.transform_to_meter(goal_x, goal_y)
        else:
            goal_x, goal_y = self.get_next_tag()
        print'send next_goal'
        print 'goal_x', goal_x
        print 'goal_y', goal_y

        resp = TagServiceResponse()
        resp.tags_x = goal_x
        resp.tags_y = goal_y
        return TagServiceResponse()

    def set_tag_stat(self, tag_index, stat):
        pass

    def find_taf_index(self):

    def get_next_tag(self):
        for i in range(TAG_STAT_OPEN, TAG_STAT_FOUND):
            minDist = 100000.0
            minDistIndex = -1
            for j in range(0, len(self._active_tags)):
                # search next tag with smallest distance
                if self._active_tags[j] == i:
                    if minDist > self._distance_values[self._last_send_tag_index][j]:
                        minDist = self._distance_values[self._last_send_tag_index][j]
                        minDistIndex = j
            if minDistIndex != -1:
                self._last_send_tag_index = minDistIndex
                self.top_ser_pub(self._my_found_tags_x[minDistIndex], self._my_found_tags_y[minDistIndex])
                return self._my_found_tags_x[minDistIndex], self._my_found_tags_y[minDistIndex]
        return self._start_x_coord, self._start_y_coord

    def save_tags_callback(self, data):
        print 'got calback data:', data
        for i in range(0, len(data.x_values)):
            self._my_found_tags_x.append(data.x_values[i])
            self._my_found_tags_y.append(data.y_values[i])
        self.write_to_file(self._my_found_tags_x, self._my_found_tags_y)

    def transform_to_pos(self, m_x, m_y):
        pos_x = np.int((m_x - self._offset_x) / self._resolution)
        pos_y = np.int((m_y - self._offset_y) / self._resolution)
        return pos_x, pos_y

    def transform_to_meter(self, pos_x, pos_y):
        m_x = pos_x * self._resolution + self._offset_x
        m_y = pos_y * self._resolution + self._offset_y
        return m_x, m_y

    def write_to_file(self, x_data, y_data):
        print 'write file'
        i = 0
        filenamex = "x.txt"
        filenamey = "y.txt"
        xfile = open(filenamex, 'w+')
        yfile = open(filenamey, 'w+')
        while i < len(x_data):
            xfile.write(x_data[i])
            xfile.write('\n')
            yfile.write(y_data[i])
            yfile.write('\n')
        xfile.close()
        yfile.close()

    def read_tags_from_file(self):
        i = 0
        self._my_found_tags_x = []
        self._my_found_tags_y = []
        filenamex = "x.txt"
        filenamey = "y.txt"
        xfile = open(filenamex, 'r')
        yfile = open(filenamey, 'r')
        strx = xfile.readline()
        stry = yfile.readline()
        while len(strx) is not 0:
            self._my_found_tags_x.append(float(strx))
            self._my_found_tags_y.append(float(stry))
            strx = xfile.readline()
            stry = yfile.readline()
        xfile.close()
        yfile.close()

    def find_first_tag(self, robot_pos_x, robot_pos_y):
        print 'find first tag'
        occupancy_grid = rospy.wait_for_message("map", OccupancyGrid)
        meta_data = occupancy_grid.info
        occupancy_map = occupancy_grid.data
        trimmed_map = np.array(occupancy_map)
        map_height = meta_data.height
        map_width = meta_data.width
        current_map = trimmed_map.reshape((map_width, map_height))
        #set value of tags to 100
        for i in range(0, len(self._my_found_tags_x)):
            x, y = self.transform_to_pos(self._my_found_tags_x[i], self._my_found_tags_y[i])
            current_map[y][x] = 100
        start_pose = np.array([robot_pos_x, robot_pos_y])
        path = [start_pose]
        closed_list = []
        first_run = True
        set_to_zero = False
        while len(path) > 0:
            current_path = path.pop(0)
            if len(path) == 0 and not first_run:
                print "bfs len 0"
            closed_list.append(current_path)
            current_x = current_path[0]
            current_y = current_path[1]
            # is wall
            if current_map[current_y, current_x] == 1:
                continue
            # unknown cell found
            if current_map[current_y, current_x] == 100:
                self.find_index_set_last(current_x, current_y)
                return current_x, current_y
            # add all neighbours of current cell
            directions = np.array([[current_x - 1, current_y], [current_x + 1, current_y],
                                   [current_x, current_y - 1], [current_x, current_y + 1]])
            np.random.shuffle(directions)
            for i in directions:
                if not self.cointains_pos(i, closed_list):
                    if not self.cointains_pos(i, path):
                        path.append(i)
        print "return start pose"
        return self._start_x_coord, self._start_y_coord

    def find_index_set_last(self, xp, yp):
        xm, ym = self.transform_to_meter(xp, yp)
        xu = xm + FIRST_TAG_TOLERANCE/2
        xl = xm - FIRST_TAG_TOLERANCE/2
        yu = ym + FIRST_TAG_TOLERANCE/2
        yl = ym - FIRST_TAG_TOLERANCE/2
        for i in range(0, len(self._my_found_tags_x)):
            if xl <= self._my_found_tags_x[i] <= xu:
                if yl <= self._my_found_tags_y[i] <= yu:
                    self._active_tags[i] = TAG_STAT_SER
                    self._last_send_tag_index = i

    def calculate_distances(self):
        print 'Calculating distances'
        occupancy_grid = rospy.wait_for_message("map", OccupancyGrid)
        meta_data = occupancy_grid.info
        occupancy_map = occupancy_grid.data
        trimmed_map = np.array(occupancy_map)
        map_height = meta_data.height
        map_width = meta_data.width
        current_map = trimmed_map.reshape((map_width, map_height))
        # distances from each tag to each others
        self._distance_values = np.array(np.shape(len(self._my_found_tags_x), len(self._my_found_tags_x)))
        # index of start values
        for i in range(0, len(self._my_found_tags_x)):
            print 'Calculation in progress Tagnr: ', i
            cxi, cyi = self.transform_to_pos(self._my_found_tags_x[i], self._my_found_tags_y[i])
            start_pose = np.array([cxi, cyi])
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
                directions = np.array([[current_x - 1, current_y], [current_x + 1, current_y],
                                       [current_x, current_y - 1], [current_x, current_y + 1]])
                np.random.shuffle(directions)
                for x in directions:
                    if not self.cointains_pos(x, closed_list):
                        if not self.cointains_pos(x, path):
                            path.append(x)
                distance += 1
            for j in range(0, len(self._my_found_tags_x)):
                cxj, cyj = self.transform_to_pos(self._my_found_tags_x[j], self._my_found_tags_y[j])
                p1 = distance_map[cxi][cyi]
                p2 = distance_map[cxj][cyj]
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
