#!/usr/bin/env python

import os
import sys
import time
import traceback
import actionlib
import matplotlib.pyplot as plt
import numpy as np
import rospy
from actionlib import SimpleGoalState
from actionlib_msgs.msg import GoalStatus
from correct_pos_srv.srv import *
from std_msgs.msg import Int16, Int16MultiArray
from explore_labyrinth_srv.srv import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, PoseWithCovarianceStamped
from map_tag_handler_srv.srv import *
from sensor_msgs.msg import LaserScan
from playsound import playsound

# specify directions
EVERYWHERE = 0
FRONT_LEFT = 1
FRONT_RIGHT = 4
BACK_LEFT = 3
BACK_RIGHT = 2

PI = 3.1415926535897
VEL_STRAIGHT = 0.15
TIME_STRAIGHT = 2
TAG_POSE_DEVIATION = 0.05    # cm !!
GOAL_POSE_DEVIATION = 0.3
GOAL_MIN_DIST_TO_TAG = 15   # *5=cm abstand
GOAL_MIN_DIST_TO_WALL = 8

STAT_FIND_POS = 'find_pos'
STAT_COLLECT_TAGS = 'collect_tags'
STAT_CHECK_TOKEN = 'check_token'

if os.name == 'nt':
    pass
else:
    import termios

class MovementController:

    def __init__(self):
        self._current_tag_x = None
        self._current_tag_y = None
        self._move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self._move_base_client.wait_for_server()
        print 'move base server connected'
        self._tag_service = rospy.ServiceProxy('get_next_Tag', TagService, headers=None)
        self._status = STAT_FIND_POS
        self._turtlebot_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self._ranges = None
        self._lidar_data = rospy.Subscriber('scan', LaserScan, self.lidar_callback)
        rospy.wait_for_message('scan', LaserScan)
        self._ninety_degree_index = len(self._ranges) / 4
        self._oneigthy_degree_index = len(self._ranges) / 2
        self._twoseventy_degree_index = self._ninety_degree_index + self._oneigthy_degree_index
        self._threesixty_degree_index = len(self._ranges)
        self._lidar_data = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.amcl_pose_callback)
        self._covariance = None
        rospy.wait_for_message('amcl_pose', PoseWithCovarianceStamped)
        self._current_pose = None
        self._current_pos_x = 0.0
        self._current_pos_y = 0.0

        self.get_correct_pose_tag_srv = rospy.ServiceProxy('drive_on_tag', CorrectPosSrv, headers=None)
        self._current_mb_goal_x = 0.0
        self._current_mb_goal_y = 0.0
        print 'init finished'

    def lidar_callback(self, data):
        self._ranges = np.array(data.ranges)
        self._ranges[self._ranges >= 2.0] = 2.0
        self._ranges[self._ranges <= 0.20] = 0.0

    def amcl_pose_callback(self, msg):
        msg = msg.pose
        cv = np.array(msg.covariance)
        self._covariance = cv.reshape((6, 6))
        self._current_pose = msg.pose
        self._current_pos_x = msg.pose.position.x
        self._current_pos_y = msg.pose.position.y

    def move_straight(self, speed):
        twist = Twist()
        twist.linear.x = speed
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self._turtlebot_pub.publish(twist)

    """
    speed_x: speed of robot in x axis
    speed_angle: rotationspeed in Degree
    angle: angle to reach in360 degree
    """
    def rotate_robot(self, speed, speed_angle, angle):
        angular_speed = speed_angle * 2 * PI / 360
        relative_angle = angle * 2 * PI / 360
        twist = Twist()
        twist.linear.x = speed
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular_speed
        t0 = rospy.Time.now().to_sec()
        current_angle = 0.0
        self._turtlebot_pub.publish(twist)
        while current_angle < relative_angle:
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed * (t1 - t0)
        self.stop_turtlebot()
        return True

    def stop_turtlebot(self):
        for i in range(0, 12):
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self._turtlebot_pub.publish(twist)

    def stop_move_base(self):
        # print self._move_base_client.get_state()
        if self._move_base_client.get_state() is GoalStatus.ACTIVE or GoalStatus.PENDING:
            self._move_base_client.cancel_all_goals()

    def control_loop(self):
        ea_bound = 0.3
        token_reached = True
        camera_tag_found = True
        while not rospy.is_shutdown():
            ea = self.calc_elips_area(self._covariance)
            ea = abs(ea)
            print 'ea: ', ea
            print 'ea_bound: ', ea_bound
            if ea >= ea_bound:
                self._status = STAT_FIND_POS
            elif ea < ea_bound:
                self._status = STAT_COLLECT_TAGS
            if self._status == STAT_FIND_POS:
                ea_bound = 1.5
                self.find_pose()
            if self._status == STAT_COLLECT_TAGS:
                if self.is_current_pos_tag_pos():
                    playsound('/home/christoph/catkin_ws/src/movement_controler/nodes/R2D2.mp3')
                    time.sleep(2)
                if self.is_current_pos_tag_pos():
                    req = TagServiceRequest()
                    if camera_tag_found:
                        req.current_pose_x = self._current_pos_x
                        req.current_pose_y = self._current_pos_y
                    else:
                        req.current_pose_x = -100.0
                        req.current_pose_y = -100.0
                    response = self._tag_service(req)
                    print'response: ', response
                    self._current_tag_x = response.tags_x
                    self._current_tag_y = response.tags_y
                    # get pos in defined dist to token
                    self._current_mb_goal_x, self._current_mb_goal_y = self.get_away(self._current_tag_x,
                                                                                     self._current_tag_y,
                                                                                     self.get_map())
                self.send_goal_to_move_base()
                result = self._move_base_client.wait_for_result(rospy.Duration.from_sec(40))
                # check for reached pos
                print 'cpose = gpose: ', self.is_current_pos_goal_pos()
                if self.is_current_pos_goal_pos():
                    req = CorrectPosSrvRequest()
                    req.stat = STAT_CHECK_TOKEN
                    req.searched_tag_x = self._current_tag_x
                    req.searched_tag_y = self._current_tag_y
                    resp = self.get_correct_pose_tag_srv(req)
                    if resp.correct_y == -100.0 and resp.correct_x == -100:
                        camera_tag_found = False
                    else:
                        camera_tag_found = True
                        self._current_mb_goal_x = resp.correct_x
                        self._current_mb_goal_y = resp.correct_y


    def send_goal_to_move_base(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "bauwen/map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self._current_mb_goal_x
        goal.target_pose.pose.position.y = self._current_mb_goal_y
        goal.target_pose.pose.orientation.w = 1
        self._current_goal_msg = goal
        self._move_base_client.send_goal(self._current_goal_msg)

    def find_pose(self):
        dir = self.eval_dir_to_go(self._ranges)
        print dir
        if dir == FRONT_LEFT:
            self.rotate_robot(0.0, 45.0, 45.0)
        if dir == FRONT_RIGHT:
            self.rotate_robot(0.0, 45.0, 315.0)
        if dir == BACK_LEFT:
            self.rotate_robot(0.0, 45.0, 135.0)
        if dir == BACK_RIGHT:
            self.rotate_robot(0.0, 45.0, 225.0)
        self.move_straight(0.1)
        time.sleep(1.0)
        self.stop_turtlebot()
        time.sleep(1.0)

    def get_map(self):
        occupancy_grid = rospy.wait_for_message("map", OccupancyGrid)
        meta_data = occupancy_grid.info
        occupancy_map = occupancy_grid.data
        trimmed_map = np.array(occupancy_map)
        self._map_height = meta_data.height
        self._map_width = map_width = meta_data.width
        current_map = trimmed_map.reshape((self._map_width, self._map_height))
        return current_map

    def get_away(self, pos_x, pos_y, current_map):
        pos_x, pos_y = self.transform_to_pos(pos_x, pos_y)
        start_pose = np.array([pos_x, pos_y])
        open_list = [start_pose]
        closed_list = []
        dist_list = [0]
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
                if distance > GOAL_MIN_DIST_TO_TAG:
                    if self.check_goal_pos(current_x, current_y, current_map):
                        return self.transform_to_meter(current_x, current_y)
            directions = np.array([[current_x - 1, current_y], [current_x + 1, current_y],
                                   [current_x, current_y - 1], [current_x, current_y + 1]], dtype=int)
            np.random.shuffle(directions)
            for x in directions:
                if not self.cointains_pos(x, closed_list):
                    if not self.cointains_pos(x, open_list):
                        open_list.append(x)
                        dist_list.append(distance)
        print 'calculation done'
        return self._current_pos_x, self._current_pos_y

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

    def cointains_pos(self, array, array_array):
        for i in array_array:
            if i[0] == array[0]:
                if i[1] == array[1]:
                    return True
        return False

    def transform_to_pos(self, m_x, m_y):
        pos_x = np.int((m_x - (-10.0)) / 0.05)
        pos_y = np.int((m_y - (-10.0)) / 0.05)
        return pos_x, pos_y

    def transform_to_meter(self, pos_x, pos_y):
        m_x = np.float(pos_x) * 0.05 + (-10.0)
        m_y = np.float(pos_y) * 0.05 + (-10.0)
        return m_x, m_y

    def is_current_pos_tag_pos(self):
        b = False
        if self._current_tag_x is not None and self._current_tag_y is not None:
            xu = self._current_tag_x + TAG_POSE_DEVIATION / 2
            xl = self._current_tag_x - TAG_POSE_DEVIATION / 2
            yu = self._current_tag_y + TAG_POSE_DEVIATION / 2
            yl = self._current_tag_y - TAG_POSE_DEVIATION / 2
            if xl <= self._current_pos_x <= xu:
                if yl <= self._current_pos_y <= yu:
                    b = True
        return b

    def is_current_pos_goal_pos(self):
        b = False
        if self._current_tag_x and self._current_tag_y != 0.0:
            xu = self._current_mb_goal_x + GOAL_POSE_DEVIATION / 2
            xl = self._current_mb_goal_x - GOAL_POSE_DEVIATION / 2
            yu = self._current_mb_goal_y + GOAL_POSE_DEVIATION / 2
            yl = self._current_mb_goal_y- GOAL_POSE_DEVIATION / 2
            if xl <= self._current_pos_x <= xu:
                if yl <= self._current_pos_y <= yu:
                    b = True
        return b

    def eval_dir_to_go(self, ranges):
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

    def calc_elips_area(self, cov):
        c = cov[0:2, 0:2]
        eig_val, eig_vec = np.linalg.eig(np.linalg.inv(c))
        eigen_x, eigen_y = eig_vec[:, 0]
        deg = np.degrees(np.arctan2(eigen_y.real, eigen_x.real))
        a, b = 2 / np.sqrt(eig_val)
        return a * b * deg


def main():
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('movement_controler')
    try:
        mc = MovementController()
        mc.control_loop()
    except Exception as e:
        print e
        traceback.print_exc()


if __name__ == "__main__":
    main()
