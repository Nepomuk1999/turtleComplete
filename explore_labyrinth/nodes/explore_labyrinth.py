#!/usr/bin/env python

import os
import sys
import time
import traceback
import actionlib
import matplotlib.pyplot as plt
import numpy as np
import rospy
from explore_labyrinth_srv.srv import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry

if os.name == 'nt':
    pass
else:
    import termios

GOAL_MIN_DIST_TO_WALL = 8
ROBOT_KNOWN_SPACE = 6


# Publisher
class LabyrinthExplorer:

    def __init__(self):
        self._pub = rospy.Publisher('/explorer_goal_pos_result', MoveBaseGoal, queue_size=10)
        print 'publisher initialized'
        self.map_trimmer = MapTrimmer()
        self._occupancy_grid = rospy.wait_for_message('/map', OccupancyGrid)
        self._occupancy_map = self._occupancy_grid.data
        self._offset_x = self._occupancy_grid.info.origin.position.x
        self._offset_y = self._occupancy_grid.info.origin.position.y
        self._resolution = self._occupancy_grid.info.resolution
        self._map_height = self._occupancy_grid.info.height
        self._map_width = self._occupancy_grid.info.width
        # reshape map
        trimmed_map = np.array(self._occupancy_map)
        self._occupancy_map = trimmed_map.reshape((self._map_width, self._map_height))
        self._odom_sub = rospy.Subscriber('/odom', Odometry, self.pose_callback)
        self._current_pose = None
        self._current_x = None
        self._current_y = None
        while self._current_pose is None:
            time.sleep(2)
        self._start_x, self._start_y = self.transform_to_pos(self._current_pose.position.x, self._current_pose.position.y)
        # self._as = actionlib.SimpleActionServer('/explorer_goal_pos', MoveBaseAction,
        #                                         execute_cb=self.movementcontroller, auto_start=False)
        # self._as.start()
        self._as = rospy.Service('/explorer_goal_pos', ExploreLabyrinth, self.movementcontroller)

    def pose_callback(self, msg):
        self._current_pose = msg.pose.pose
        self._current_x = self._current_pose.position.x
        self._current_y = self._current_pose.position.y

        self._current_x, self._current_y = self.transform_to_pos(self._current_x, self._current_y)
        # print msg.pose.pose

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

        # Plot heatmap of trimmed map
        # current_map[robot_pos_y, robot_pos_x] = 4
        # plt.imshow(current_map, cmap='hot', interpolation='nearest')
        # plt.show()
        first_run = True
        while len(path) > 0:
            if first_run:
                robot_pos_x = robot_pos_x + 2
            current_path = path.pop(0)
            if len(path) == 0 and not first_run:
                print "bfs len 0"
                # f = plt.figure(1)
                # f.imshow(current_map, cmap='hot', interpolation='nearest')
                # f.show()
            closed_list.append(current_path)
            current_x = current_path[0]
            current_y = current_path[1]
            # is wall

            if current_map[current_y, current_x] == 1:
                continue
            # unknown cell found
            if current_map[current_y, current_x] == -1:
                unknown_x = current_x
                unknown_y = current_y
                cx, cy = self.next_known_cell(current_x, current_y, current_map)
                # current_map[robot_pos_y, robot_pos_x] = 5
                # current_map[cy, cx] = 10
                # plt.imshow(current_map, cmap='hot', interpolation='nearest')
                # plt.show()
                return cx, cy, unknown_x, unknown_y
            # add all neighbours of current cell
            directions = np.array([[current_x - 1, current_y], [current_x + 1, current_y],
                                   [current_x, current_y - 1], [current_x, current_y + 1]])
            np.random.shuffle(directions)
            for i in directions:
                if not self.cointains_pos(i, closed_list):
                    if not self.cointains_pos(i, path):
                        path.append(i)
            first_run = False
            current_map[current_y][current_x] = 10
        print "return start pose"
        return self._start_x, self._start_y, 0, 0

    def cointains_pos(self, array, array_array):
        for i in array_array:
            if i[0] == array[0]:
                if i[1] == array[1]:
                    return True
        return False

    def next_known_cell(self, pos_x, pos_y, current_map):

        # plt.imshow(current_map, cmap='hot', interpolation='nearest')
        # plt.show()
        start_pose = np.array([pos_x, pos_y])
        path = [start_pose]
        closed_list = []
        first_run = True
        while len(path) > 0:
            if len(path) == 0 and not first_run:
                print "next known cell len 0"
                f = plt.figure(1)
                plt.imshow(current_map, cmap='hot', interpolation='nearest')
                f.show()
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
            #if current_map[current_y, current_x] == 0:
            #    return self.goal_pos_correction(current_x, current_y, current_map)

            # add all neighbours of current cell
            directions = np.array([[current_x + 1, current_y], [current_x - 1, current_y],
                                   [current_x, current_y + 1], [current_x, current_y - 1]])
            np.random.shuffle(directions)
            for i in directions:
                if not self.cointains_pos(i, closed_list):
                    if not self.cointains_pos(i, path):
                        path.append(i)
            first_run = False

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

    def movementcontroller(self, goal):
        print 'calc next pos'
        # get map to avoid update while processing
        self._occupancy_grid = rospy.wait_for_message("/map", OccupancyGrid)
        self.update_map_data(self._occupancy_grid)
        # self.update_map_data(self._occupancy_grid)
        cleared_map = self.map_trimmer.trim_map(self._occupancy_map, self._current_x, self._current_y,
                                                self._map_height, self._map_width)
        next_x, next_y, unknown_x, unknown_y = self.bfs(cleared_map, self._current_x, self._current_y)
        # f = plt.figure(1)
        # plt.imshow(cleared_map, cmap='hot', interpolation='nearest')
        # plt.show()
        # cleared_map[unknown_y, unknown_x] = 10
        # cleared_map[self._current_y, self._current_x] = 5
        # cleared_map[next_y, next_x] = 10
        # f = plt.figure(2)
        # plt.imshow(cleared_map, cmap='hot', interpolation='nearest')
        # plt.show()

        next_xm, next_ym = self.transform_to_meter(next_x, next_y)
        return ExploreLabyrinthResponse(next_xm, next_ym)

    # def publish_goal(self, x_goal, y_goal):
    #     goal = MoveBaseGoal()
    #     goal.target_pose.header.frame_id = "/map"
    #     goal.target_pose.header.stamp = rospy.Time.now()
    #     goal.target_pose.pose.position.x = x_goal
    #     goal.target_pose.pose.position.y = y_goal
    #     goal.target_pose.pose.orientation.w = 1
    #     return goal


class MapTrimmer:

    def __init__(self):
        pass

    def trim_map(self, untrimmed_map, pos_x, pos_y, _map_height, _map_width):

        # set values of 100 to 1 for walls
        untrimmed_map[untrimmed_map == 100] = 1

        # plt.imshow(trimmed_map, cmap='hot', interpolation='nearest')
        # plt.show()

        # inflate walls
        trimmed_map = self.inflate_wals(untrimmed_map, 0)
        new_trimmed_map = self.fix_robot_pos(pos_x, pos_y, _map_height, _map_width, trimmed_map)

        # # Plot heatmap of trimmed map
        #plt.imshow(trimmed_map, cmap='hot', interpolation='nearest')
        #plt.show()
        #time.sleep(2)

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
        to_be_inflated = np.where(trimmed_map == 5)
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
