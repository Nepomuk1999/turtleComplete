#!/usr/bin/env python

import os
import sys
import time
import traceback
import actionlib
import cv2
import matplotlib
matplotlib.use('TkAgg')
from matplotlib import pyplot as plt
import numpy as np
import rospy
from explore_labyrinth_srv.srv import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist, Pose, PointStamped, Point
from scipy import ndimage, misc

if os.name == 'nt':
    pass
else:
    import termios

GOAL_MIN_DIST_TO_WALL = 8
ROBOT_KNOWN_SPACE = 10
MASK_DISTANCE = 12


# Publisher
class LabyrinthExplorer:

    def __init__(self):
        self._callback_counter = 0
        self._node_use_counter = 1
        self.match_divider = 150
        self._pub = rospy.Publisher('explorer_goal_pos_result', MoveBaseGoal, queue_size=10)
        print 'publisher initialized'
        self.map_trimmer = MapTrimmer()
        self._occupancy_grid = rospy.wait_for_message('map', OccupancyGrid)
        self._occupancy_map = self._occupancy_grid.data
        self._offset_x = self._occupancy_grid.info.origin.position.x
        self._offset_y = self._occupancy_grid.info.origin.position.y
        self._resolution = self._occupancy_grid.info.resolution
        self._map_height = self._occupancy_grid.info.height
        self._map_width = self._occupancy_grid.info.width
        self._seen_map = np.ones((self._map_width, self._map_height), dtype=int)
        self._seen_map[self._seen_map == 1] = -1
        #self._seen_map = np.multiply(self._seen_map, self._occupancy_map)
        # reshape map
        trimmed_map = np.array(self._occupancy_map)
        self._occupancy_map = trimmed_map.reshape((self._map_width, self._map_height))
        self._pose_pub_sub = rospy.Subscriber('robot_pose', Pose, self.pose_callback)
        self._current_pose = None
        self._current_x = None
        self._current_y = None
        rospy.wait_for_message('robot_pose', Pose)
        self._start_x, self._start_y = self.transform_to_pos(self._current_pose.position.x, self._current_pose.position.y)
        self._as = rospy.Service('explorer_goal_pos', ExploreLabyrinth, self.movementcontroller)

    def pose_callback(self, msg):
        self._callback_counter = self._callback_counter + 1
        self._current_pose = msg
        self._current_x = self._current_pose.position.x
        self._current_y = self._current_pose.position.y
        self._current_x, self._current_y = self.transform_to_pos(self._current_x, self._current_y)
        #if self._callback_counter % 2 == 0:
        self.update_seen_map(self._current_pose.orientation)

    def update_seen_map(self, orientation):
        phi = self.get_rotation(orientation)
        phi = phi/np.pi * 180
        blobb = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]], dtype=int)
        blobb_angle = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                [0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]], dtype=int)
        # front
        rotated = blobb
        if -23.0 <= phi < 23.0:
            rotated = blobb
        elif 23.0 <= phi < 68.0:
            rotated = blobb_angle
        # left
        elif 68.0 <= phi < 113.0:
            rotated = np.rot90(blobb)
        elif 113.0 <= phi < 158.0:
            rotated = np.rot90(blobb_angle)
        # back
        elif 158.0 <= phi <= 180.0 or -180.0 <= phi <= -158.0:
            rotated = np.rot90(np.rot90(blobb))
        elif -158.0 <= phi < -113.0:
            rotated = np.rot90(np.rot90(blobb_angle))
        # rigth
        elif -68.0 <= phi < -113.0:
            rotated = np.rot90(np.rot90(np.rot90(blobb)))
        elif -68.0 <= phi < -23:
            rotated = np.rot90(np.rot90(np.rot90(blobb_angle)))
        h, w = np.shape(rotated)
        h = h/2
        x_lower_b = self._current_x - h
        y_lower_b = self._current_y - h
        x_upper_b = self._current_x + (h + 1)
        y_upper_b = self._current_y + (h + 1)
        self._seen_map[y_lower_b:y_upper_b, x_lower_b:x_upper_b] = \
            np.add(self._seen_map[y_lower_b:y_upper_b, x_lower_b:x_upper_b], rotated)


    def get_rotation(self, orientation_q):
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        return yaw

    def transform_to_pos(self, m_x, m_y):
        pos_x = np.int((m_x - self._offset_x) / self._resolution)
        pos_y = np.int((m_y - self._offset_y) / self._resolution)
        return pos_x, pos_y

    def transform_to_meter(self, pos_x, pos_y):
        m_x = pos_x * self._resolution + self._offset_x
        m_y = pos_y * self._resolution + self._offset_y
        return m_x, m_y

    def bfs(self, current_map, robot_pos_x, robot_pos_y):
        start_pose = np.array([robot_pos_x, robot_pos_y])
        path = [start_pose]
        map_size_y, map_size_x = np.shape(current_map)
        closed_list = []
        first_run = True
        set_to_zero = False
        while len(path) > 0:
            # if first_run:
            #     robot_pos_x = robot_pos_x + 2
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
            if current_map[current_y, current_x] == -1:
                if self.match_divider == 1:
                    current_map, set_to_zero = self.mark_small_unseen_as_seen(current_map, current_x, current_y)
                if not set_to_zero:
                    cx, cy = self.next_known_cell(current_x, current_y, current_map)
                    if self.match_divider == 1 and (cy == self._start_y and cx == self._start_x):
                        return self._start_x, self._start_y,
                    if cy == self._start_y and cx == self._start_x:
                        print 'set to robot pose'
                        self.match_divider = 1
                        cx = self._current_x
                        cy = self._current_y
                    return cx, cy
            # add all neighbours of current cell
            directions = np.array([[current_x - 1, current_y], [current_x + 1, current_y],
                                   [current_x, current_y - 1], [current_x, current_y + 1]])
            np.random.shuffle(directions)
            for i in directions:
                if not self.cointains_pos(i, closed_list):
                    if not self.cointains_pos(i, path):
                        path.append(i)
        first_run = False

        print "return start pose"
        if self.match_divider != 1:
            print 'set to cp'
            self.match_divider = 1
            cx = self._current_x
            cy = self._current_y
            return cx, cy
        return self._start_x, self._start_y

    def cointains_pos(self, array, array_array):
        for i in array_array:
            if i[0] == array[0]:
                if i[1] == array[1]:
                    return True
        return False

    def next_known_cell(self, pos_x, pos_y, current_map):
        start_pose = np.array([pos_x, pos_y])
        path = [start_pose]
        closed_list = []
        first_run = True
        while len(path) > 0:
            current_path = path.pop(0)
            closed_list.append(current_path)
            current_x = current_path[0]
            current_y = current_path[1]
            # is wall
            if current_map[current_y, current_x] == 1 or (current_map[current_y, current_x] == -1 and not first_run):
                continue
            # known cell found
            if self.check_goal_pos(current_x, current_y, current_map):
                return current_x, current_y
            directions = np.array([[current_x + 1, current_y], [current_x - 1, current_y],
                                   [current_x, current_y + 1], [current_x, current_y - 1]])
            np.random.shuffle(directions)
            for i in directions:
                if not self.cointains_pos(i, closed_list):
                    if not self.cointains_pos(i, path):
                        path.append(i)
            first_run = False
        print 'no next known cell found'
        return self._start_x, self._start_y

    def check_goal_pos(self, pos_x, pos_y, current_map):
        lower_x = pos_x - (np.int(GOAL_MIN_DIST_TO_WALL / 2))
        if lower_x < 0:
            lower_x = 0
        upper_x = pos_x + (np.int(GOAL_MIN_DIST_TO_WALL / 2))
        if upper_x > self._map_width - 1:
            upper_x = self._map_width - 1

        lower_y = pos_y - (np.int(GOAL_MIN_DIST_TO_WALL / 2))
        if lower_y < 0:
            lower_y = 0
        upper_y = pos_y + (np.int(GOAL_MIN_DIST_TO_WALL / 2))
        if upper_y > self._map_height - 1:
            upper_y = self._map_height - 1
        current_blobb = current_map[lower_y:upper_y + 1, lower_x:upper_x + 1]
        if np.sum(np.absolute(current_blobb)) == 0:
            return True
        else:
            return False

    def update_map_data(self, occupancy_grid):
        self._occupancy_map = occupancy_grid.data
        meta_data = occupancy_grid.info
        self._offset_x = meta_data.origin.position.x
        self._offset_y = meta_data.origin.position.y
        self._resolution = meta_data.resolution
        self._map_height = meta_data.height
        self._map_width = meta_data.width
        # reshape map
        trimmed_map = np.array(self._occupancy_map)
        self._occupancy_map = trimmed_map.reshape((self._map_width, self._map_height))

    def match_maps(self, current_map):
        for x in range(0, self._map_width):
            for y in range(0, self._map_height):
                if (current_map[y][x] == 0) and (self._seen_map[y][x] == -1):
                    current_map[y][x] = -1
        print 'maps matched'
        current_map = self.inflate_wals_seen(current_map, 2)
        return current_map

    def inflate_wals_seen(self, trimmed_map, inflation_factor):
        to_be_inflated = np.where(trimmed_map == 1)
        map_size_y, map_size_x = trimmed_map.shape
        x_list = to_be_inflated[1]
        y_list = to_be_inflated[0]
        for i in range(0, len(x_list)):
            x = x_list[i]
            y = y_list[i]
            for j in range(-inflation_factor, inflation_factor + 1):
                for k in range(-inflation_factor, inflation_factor + 1):
                    if 0 < x + j < map_size_x - 1 and 0 < y + k < map_size_y - 1:
                        if trimmed_map[y+k][x+j] != 1:
                            #TODO check if wall is in between maybe
                            trimmed_map[y+k][x+j] = 0
        return trimmed_map

    def mark_small_unseen_as_seen(self, map, x, y):
        b = False
        md = 2
        lx = x - (md / 2)
        ux = x + (md / 2)+1
        ly = y - (md / 2)
        uy = y + (md / 2)+1
        check_area = map[ly:uy, lx:ux]
        print check_area
        val_of_unseen = np.sum(check_area == -1)
        print val_of_unseen
        if val_of_unseen <= 4:
            self._seen_map[y, x] = 0
            map[y, x] = 0
            b = True
        return map, b

    def mask_current_pose_as_seen(self, map, x, y):
        lx = x - (MASK_DISTANCE/2)
        ux = x + (MASK_DISTANCE/2)
        ly = y - (MASK_DISTANCE/2)
        uy = y + (MASK_DISTANCE/2)
        for i in range(lx, ux + 1):
            for j in range(ly, uy + 1):
                if map[j, i] == -1:
                    map[j, i] = 0
        return map

    def movementcontroller(self, goal):
        print 'calc next pos'
        # get map to avoid update while processing
        self._occupancy_grid = rospy.wait_for_message("map", OccupancyGrid)
        self.update_map_data(self._occupancy_grid)
        # self.update_map_data(self._occupancy_grid)
        cleared_map = self.map_trimmer.trim_map(self._occupancy_map, self._current_x, self._current_y,
                                                self._map_height, self._map_width)
        print 'div:', self.match_divider
        print 'count:', self._node_use_counter
        if self._node_use_counter % self.match_divider == 0:
            self._seen_map[self._seen_map > 1] = 1
            cleared_map = self.match_maps(cleared_map)
            # f1 = plt.figure(1)
            # plt.imshow(self._seen_map, cmap='hot', interpolation='nearest')
            # self._seen_map[self._seen_map > 1] = 1
            # plt.show()
            # f1 = plt.figure(2)
            # plt.imshow(cleared_map, cmap='hot', interpolation='nearest')
            # plt.show()
            cleared_map = self.mask_current_pose_as_seen(cleared_map, self._current_x, self._current_y)
        next_x, next_y = self.bfs(cleared_map, self._current_x, self._current_y)
        self._node_use_counter = self._node_use_counter + 1
        if self._node_use_counter % 3 == 0:
            cleared_map[next_y, next_x] = 10
            f1 = plt.figure(2)
            plt.imshow(cleared_map, cmap='hot', interpolation='nearest')
            plt.show()
        next_xm, next_ym = self.transform_to_meter(next_x, next_y)
        return ExploreLabyrinthResponse(next_xm, next_ym)


class MapTrimmer:

    def __init__(self):
        self._first_run = True

    def trim_map(self, untrimmed_map, pos_x, pos_y, _map_height, _map_width):
        untrimmed_map[untrimmed_map == 100] = 1
        new_trimmed_map = self.inflate_wals(untrimmed_map, 0)
        if self._first_run:
            new_trimmed_map = self.fix_robot_pos(pos_x, pos_y, _map_height, _map_width, new_trimmed_map)
            self._first_run = False
        return new_trimmed_map

    def fix_robot_pos(self, pos_x, pos_y, _map_height, _map_width, current_map):
        lower_x = pos_x - (np.int(ROBOT_KNOWN_SPACE / 2))
        if lower_x < 0:
            lower_x = 0
        upper_x = pos_x + (np.int(ROBOT_KNOWN_SPACE / 2))
        if upper_x > _map_width - 1:
            upper_x = _map_width - 1

        lower_y = pos_y - (np.int(ROBOT_KNOWN_SPACE / 2))
        if lower_y < 0:
            lower_y = 0
        upper_y = pos_y + (np.int(ROBOT_KNOWN_SPACE / 2))
        if upper_y > _map_height - 1:
            upper_y = _map_height - 1
        for i in range(lower_x, upper_x + 2):
            for j in range(lower_y, upper_y + 2):
                current_map[j][i] = 0
        return current_map

    def inflate_wals(self, trimmed_map, inflation_factor):
        to_be_inflated = np.where(trimmed_map == 1)
        map_size_y, map_size_x = trimmed_map.shape
        x_list = to_be_inflated[1]
        y_list = to_be_inflated[0]
        for i in range(0, len(x_list)):
            x = x_list[i]
            y = y_list[i]
            for j in range(0 - inflation_factor, inflation_factor + 1):
                for k in range(0 - inflation_factor, inflation_factor + 1):
                    if 0 < x + j < map_size_x - 1 and 0 < y + k < map_size_y - 1:
                        trimmed_map[y+k][x+j] = 1
        return trimmed_map


def main():
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('explore_labyrinth')
    try:
        LabyrinthExplorer()
        rospy.spin()
    except Exception as e:
        print e
        traceback.print_exc()



if __name__ == "__main__":
    main()
