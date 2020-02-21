#!/usr/bin/env python

import os
from os.path import expanduser
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
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import subprocess

# specify directions
FRONT_LEFT = 0
Front_RIGHT = 1
BACK_LEFT = 2
BACK_RIGHT = 3

PI = 3.1415926535897
MARKER_SCALE = 0.05
STAT_SEARCH = 'search_tokens'
STAT_DRIVE = 'drive_to_tokens'
STAT_READY = 'ready'

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

        self._marker_array = MarkerArray()
        self._marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)

        top_ser = rospy.get_param('~topic_searching', default='searching')
        top_rea = rospy.get_param('~topic_reached', default='reached')
        self._search_pub = rospy.Publisher(top_ser, Point, queue_size=10)
        self._search_sub = rospy.Subscriber(top_ser, Point, self.top_ser_callback)
        self._reached_pub = rospy.Publisher(top_rea, Point, queue_size=10)
        self._reached_sub = rospy.Subscriber(top_rea, Point, self.top_rea_callback)

        self._start_x_coord = 0.0
        self._start_y_coord = 0.0
        self._last_x_coord = 0.0
        self._last_y_coord = 0.0

        self._tags_stats = []
        self._my_found_tags_x = []
        self._my_found_tags_y = []

        self._last_send_tag_index = -1

        self._distance_values = None
        self._stat = STAT_READY
        print 'end init'

    def print_state(self):
        print '--------state--------------------------'
        print 'self._tags_stats: ', self._tags_stats
        print 'self._my_found_tags_x =: ', self._my_found_tags_x
        print 'self._my_found_tags_y: ', self._my_found_tags_y
        print 'self._last_send_tag_index: ', self._last_send_tag_index
        print 'self._distance_values: ', self._distance_values
        print '--------state--------------------------'

    def top_ser_callback(self, data):
        print 'callback search: ', data
        x = data.x
        y = data.y
        i = self.find_tag_index(x, y)
        self.set_tag_stat(i, TAG_STAT_SER)
        self.update_marker_index(i, 1.0, 1.0, 0.0)

    def top_ser_pub(self, x, y):
        p = Point()
        p.x = x
        p.y = y
        self._search_pub.publish(p)

    def top_rea_callback(self, data):
        print 'callback reached:', data
        x = data.x
        y = data.y
        i = self.find_tag_index(x, y)
        self.set_tag_stat(i, TAG_STAT_FOUND)
        self.update_marker_index(i, 0.0, 1.0, 0.0)

    def top_rea_pub(self, x, y):
        print 'pub reached:', x, y
        p = Point()
        p.x = 1000
        p.y = y
        self._reached_pub.publish(p)

    def provide_next_tag(self, msg):
        if self.call_counter == 0:
            self.read_tags_from_file()
            filenamedist = expanduser("~/catkin_ws/src/map_tag_handler/nodes/distances.txt")
            self._distance_values = np.loadtxt(filenamedist, delimiter=', ')
            self.print_state()
            self.update_marker_array(self._my_found_tags_x, self._my_found_tags_y)
            print 'first call'
            print 'start_x:', msg.current_pose_x
            print 'start_y:', msg.current_pose_y
            self._start_x_coord, self._start_y_coord = self.transform_to_pos(msg.current_pose_x, msg.current_pose_y)
            goal_x, goal_y = self.find_first_tag(self._start_x_coord, self._start_y_coord)
            goal_x, goal_y = self.transform_to_meter(goal_x, goal_y)
            i = self.find_tag_index(goal_x, goal_y)
            goal_x = self._my_found_tags_x[i]
            goal_y = self._my_found_tags_y[i]
        else:
            if msg.current_pose_x is not -100.0:
                self.top_rea_pub(msg.current_pose_x, msg.current_pose_y)
            i = self.find_tag_index(msg.current_pose_x, msg.current_pose_y)
            self.set_tag_stat(i, TAG_STAT_FOUND)
            if msg.current_pose_x is not -100.0:
                goal_x, goal_y, ind = self.get_next_tag()
            else:
                goal_x = self._last_x_coord
                goal_y = self._last_y_coord
        self.call_counter = self.call_counter + 1
        print'send next_goal'
        print 'goal_x', goal_x
        print 'goal_y', goal_y
        i = self.find_tag_index(goal_x, goal_y)
        self.set_tag_stat(i, TAG_STAT_SER)
        resp = TagServiceResponse()
        resp.tags_x = goal_x
        resp.tags_y = goal_y
        self._last_x_coord = goal_x
        self._last_y_coord = goal_y
        return resp

    def set_tag_stat(self, tag_index, stat):
        if self._tags_stats[tag_index] < stat:
            self._tags_stats[tag_index] = stat

    def find_tag_index(self, tx, ty):
        xu = tx + FIRST_TAG_TOLERANCE/2
        xl = tx - FIRST_TAG_TOLERANCE/2
        yu = ty + FIRST_TAG_TOLERANCE/2
        yl = ty - FIRST_TAG_TOLERANCE/2
        for i in range(0, len(self._my_found_tags_x)):
            if xl <= self._my_found_tags_x[i] <= xu:
                if yl <= self._my_found_tags_y[i] <= yu:
                    return i
        return -1

    def get_next_tag(self):
        for i in range(1, 3):
            minDist = 100000.0
            minDistIndex = -1
            for j in range(0, len(self._tags_stats)):
                # search next tag with smallest distance
                if self._tags_stats[j] == i:
                    if minDist > self._distance_values[self._last_send_tag_index][j]:
                        minDist = self._distance_values[self._last_send_tag_index][j]
                        minDistIndex = j
            if minDistIndex != -1:
                self._last_send_tag_index = minDistIndex
                self.top_ser_pub(self._my_found_tags_x[minDistIndex], self._my_found_tags_y[minDistIndex])
                return self._my_found_tags_x[minDistIndex], self._my_found_tags_y[minDistIndex], minDistIndex
        return self._start_x_coord, self._start_y_coord, -1

    def save_tags_callback(self, data):
        print 'got calback data:', data
        for i in range(0, len(data.x_values)):
            self._my_found_tags_x.append(data.x_values[i])
            self._my_found_tags_y.append(data.y_values[i])
        self.write_to_file(self._my_found_tags_x, self._my_found_tags_y)
        self.calculate_distances()

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
        filenamex = expanduser("~/catkin_ws/src/map_tag_handler/nodes/x.txt")
        filenamey = expanduser("~/catkin_ws/src/map_tag_handler/nodes/y.txt")
        print filenamex
        xfile = open(filenamex, 'w+')
        yfile = open(filenamey, 'w+')
        print len(x_data)
        print xfile.name
        i = 0
        while i < len(x_data):
            xfile.write(str(x_data[i]))
            xfile.write('\n')
            yfile.write(str(y_data[i]))
            yfile.write('\n')
            i = i + 1
        xfile.close()
        yfile.close()
        print 'close files'
        subprocess.check_call(['rosrun', 'map_server', 'map_saver', '-f', 'map'])
        print 'map saved'

    def read_tags_from_file(self):
        i = 0
        self._my_found_tags_x = []
        self._my_found_tags_y = []
        filenamex = expanduser("~/catkin_ws/src/map_tag_handler/nodes/x.txt")
        filenamey = expanduser("~/catkin_ws/src/map_tag_handler/nodes/y.txt")
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
        print 'tx:', self._my_found_tags_x
        print 'ty:', self._my_found_tags_y
        for i in range(0, len(self._my_found_tags_y)):
            self._tags_stats.append(TAG_STAT_OPEN)

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
            current_map[y][x] = 1000
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
            if current_map[current_y, current_x] == 1000:
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
                    self._tags_stats[i] = TAG_STAT_SER
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
        self._distance_values = np.zeros((len(self._my_found_tags_x), len(self._my_found_tags_x)), dtype=int)
        # index of start values
        for i in range(0, len(self._my_found_tags_x)):
            print 'Calculation in progress Tagnr: ', i
            cxi, cyi = self.transform_to_pos(self._my_found_tags_x[i], self._my_found_tags_y[i])
            start_pose = np.array([cxi, cyi])
            open_list = [start_pose]
            closed_list = []
            dist_list = []
            dist_list.append(0)
            distance_map = np.zeros((map_width, map_height), dtype=int)
            while len(open_list) > 0:
                current_path = open_list.pop(0)
                distance = dist_list.pop(0) + 1
                closed_list.append(current_path)
                current_x = current_path[0]
                current_y = current_path[1]
                # is wall
                if current_map[current_y, current_x] == 100 or current_map[current_y, current_x] == -1:
                    continue
                if current_map[current_y, current_x] == 0:
                    distance_map[current_y][current_x] = distance
                directions = np.array([[current_x - 1, current_y], [current_x + 1, current_y],
                                       [current_x, current_y - 1], [current_x, current_y + 1]], dtype=int)
                np.random.shuffle(directions)
                for x in directions:
                    if not self.cointains_pos(x, closed_list):
                        if not self.cointains_pos(x, open_list):
                            open_list.append(x)
                            dist_list.append(distance)
            for j in range(0, len(self._my_found_tags_x)):
                cxj, cyj = self.transform_to_pos(self._my_found_tags_x[j], self._my_found_tags_y[j])
                p1 = distance_map[cyi][cxi]
                p2 = distance_map[cyj][cxj]
                dist = abs(p1 - p2)
                self._distance_values[i][j] = dist
                print 'current calculated distances:'
                print self._distance_values
        print 'calculation done'
        filenamedist = expanduser("~/catkin_ws/src/map_tag_handler/nodes/distances.txt")
        np.savetxt(filenamedist, self._distance_values, fmt="%2.3f", delimiter=', ')


    def cointains_pos(self, array, array_array):
        for i in array_array:
            if i[0] == array[0]:
                if i[1] == array[1]:
                    return True
        return False

    def update_marker_array(self, token_glob_x, token_glob_y, r=1.0, g=0.0, b=0.0):
        self._marker_array = MarkerArray()
        for i in range(0, len(token_glob_y)):
            marker = Marker()
            marker.header.frame_id = "bauwen/map"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = MARKER_SCALE
            marker.scale.y = MARKER_SCALE
            marker.scale.z = MARKER_SCALE
            marker.color.a = 1
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = token_glob_x[i]
            marker.pose.position.y = token_glob_y[i]
            marker.pose.position.z = 0.0
            self._marker_array.markers.append(marker)
        id = 0
        for m in self._marker_array.markers:
            m.id = id
            id += 1
        # Publish the MarkerArray
        self._marker_pub.publish(self._marker_array)
        rospy.sleep(0.01)

    def update_marker_index(self, index, r=1.0, g=0.0, b=0.0):
        marker = self._marker_array.markers[index]
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        self._marker_array.markers[index] = marker
        # Publish the MarkerArray
        self._marker_pub.publish(self._marker_array)
        rospy.sleep(0.01)


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
