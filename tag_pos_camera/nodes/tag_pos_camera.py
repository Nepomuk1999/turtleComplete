#!/usr/bin/env python

import os
import time
import sys
import traceback
import numpy as np
import rospy
import tf
import actionlib
import matplotlib.pyplot as plt
from scipy.ndimage import label, generate_binary_structure
from explore_labyrinth_srv.srv import *
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist, Pose, PointStamped, Point, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from correct_pos_srv.srv import *
from std_msgs.msg import String, Time
from map_tag_handler_srv.srv import *
from save_tag_msg.msg import *
from pixy_msgs.msg import *
from actionlib_msgs.msg import GoalStatus
from actionlib_msgs.msg import GoalStatus
from explore_labyrinth_srv.srv import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult

POSE_DEVIATION = 0.2
STAT_CHECK_TOKEN = 'check_token'
PI = 3.1415926535897

if os.name == 'nt':
    pass
else:
    import termios

ELEMENT_RANGE_WITH = 0.5

class CameraController:

    def __init__(self):
        self._pos_token_glob_x = []
        self._pos_token_glob_y = []
        self.blob_sub = rospy.Subscriber('block_data', PixyData, self.blobb_callback)
        self._blob_y = 0.0
        self._blob_x = 0.0

        self._last_stamp = Time()
        # self._pose_pub_sub = rospy.Subscriber('pose', PoseStamped, self.pose_pub_callback)
        self._pose_pub_sub = rospy.Subscriber('robot_pose', Pose, self.pose_pub_callback)
        self._current_pose_pub = None
        self._current_x_pub = None
        self._current_y_pub = None
        self._current_orientation_pub = None
        self._phi_pub = None
        print 'wait pose'
        # rospy.wait_for_message('pose', PoseStamped)
        rospy.wait_for_message('robot_pose', Pose)
        print 'got pose'
        self._found_x = []
        self._found_y = []
        self._interrupt_pub = rospy.Subscriber('check_token', CorrectPosSrv, self.interrupt_callback)
        msg = rospy.wait_for_message("map", OccupancyGrid)
        meta_data = msg.info
        self._offset_x = meta_data.origin.position.x
        self._offset_y = meta_data.origin.position.y
        self._resolution = meta_data.resolution
        self._tl = tf.TransformListener()

        self._turtlebot_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self._move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self._move_base_client.wait_for_server()
        self._current_goal_x = 0.0
        self._current_goal_y = 0.0
        print 'init finished'

    def blobb_callback(self, blob_data):
        stamp_nsec = blob_data.header.stamp.nsecs
        if stamp_nsec != 0:
            print stamp_nsec
            self._last_stamp = blob_data.header.stamp
            self._blob_y = blob_data.blocks[0].roi.x_offset
            self._blob_x = blob_data.blocks[0].roi.y_offset
            rc_blob_x, rc_blob_y = self.get_pose_token_robot_coord(self._blob_x, self._blob_y)
            mc_blob_x, mc_blob_y = self.get_pose_token_map(rc_blob_x, rc_blob_y, self._last_stamp)
            print  mc_blob_x, mc_blob_y
            if mc_blob_y != 0 and mc_blob_x != 0:
                print 'mc_blob_x:', mc_blob_x
                print 'mc_blob_y:', mc_blob_y
                self._found_x.append(mc_blob_x)
                self._found_y.append(mc_blob_y)

    def interrupt_callback(self, msg):
        if msg.stat == STAT_CHECK_TOKEN:
            x = 0.0
            y = 0.0
            found_correct_tag = False
            while not found_correct_tag:
                self._current_goal_x = msg.correct_x
                self._current_goal_y = msg.correct_y
                del self._found_x[:]
                del self._found_y[:]
                #rotate til tag found
                self.rotate_robot(0.0, 50.0)
                while len(self._found_x) < 2:
                    time.sleep(2)
                self.stop_turtlebot()
                #get token when robot stands still
                del self._found_x[:]
                del self._found_y[:]
                while len(self._found_x) < 2:
                    time.sleep(2)
                token_glob_x, token_glob_y = self.mean_token()
                x = token_glob_x[0]
                y = token_glob_y[0]
                # check tag pos
                found_correct_tag = self.is_current_pos_goal_pos(x, y)
            resp = CorrectPosSrvResponse()
            resp.correct_x = x
            resp.correct_x = y
            return resp

    def is_current_pos_goal_pos(self, cx, cy):
        b = False
        if self._current_goal_x and self._current_goal_y != 0.0:
            xu = self._current_goal_x + POSE_DEVIATION / 2
            xl = self._current_goal_x - POSE_DEVIATION / 2
            yu = self._current_goal_y + POSE_DEVIATION / 2
            yl = self._current_goal_y - POSE_DEVIATION / 2
            if xl <= cx <= xu:
                if yl <= cy <= yu:
                    b = True
        return b

    # for robot_pose_publisher
    def pose_pub_callback(self, msg):
        self._current_pose_pub = msg
        self._current_x_pub = self._current_pose_pub.position.x
        self._current_y_pub = self._current_pose_pub.position.y
        self._current_orientation_pub = self._current_pose_pub.orientation
        self._phi_pub = self.get_rotation(msg)

    # for pose_publisher
    # def pose_pub_callback(self, msg):
    #     self._current_pose_pub = msg.pose
    #     self._current_x_pub = self._current_pose_pub.position.x
    #     self._current_y_pub = self._current_pose_pub.position.y
    #     self._current_orientation_pub = self._current_pose_pub.orientation
    #     self._phi_pub = self.get_rotation(msg)

    # checks if array contains element of a given range
    def glob_x_y_contains_in_range(self, current_token_x, current_token_y):
        if len(self._found_x) > 0:
            for i in range(0, len(self._found_x)):
                x_element_range_lower_bound = self._found_x[i] - ELEMENT_RANGE_WITH
                x_element_range_upper_bound = self._found_x[i] + ELEMENT_RANGE_WITH
                y_element_range_lower_bound = self._found_y[i] - ELEMENT_RANGE_WITH
                y_element_range_upper_bound = self._found_y[i] + ELEMENT_RANGE_WITH
                if x_element_range_lower_bound <= current_token_x <= x_element_range_upper_bound:
                    if y_element_range_lower_bound <= current_token_y <= y_element_range_upper_bound:
                        return True
        return False

    def get_pose_token_robot_coord(self, blob_x, blob_y):
        middel_height = blob_y
        middel_width = blob_x
        point_1 = np.array([middel_height, middel_width, 1])
        H = np.array([[9.27998422577072, 19.6251228932382, -320.157395862157 ],
                      [5.24077929985246, 38.6451497322568, -201.136898122784],
                      [0.00619017665655573, 0.0317700847033603, 1]])
        point_2 = H.dot(point_1)
        py = point_2[0] / point_2[2]
        px = point_2[1] / point_2[2]
        py_robot = (1200 / 2 - py) / 1000
        px_robot = (1200 - px) / 1000
        return px_robot, py_robot

    def get_pose_token_map(self, rc_blob_x, rc_blob_y, blob_data_stamp):
        ps = PointStamped()
        ps.header.frame_id = 'bauwen/base_footprint'
        ps.header.stamp = blob_data_stamp
        ps.point = Point(x=rc_blob_x, y=rc_blob_y)
        try:
            ps_map = self._tl.transformPoint('bauwen/map', ps)
            mx = ps_map.point.x
            my = ps_map.point.y
            #Return in m
            return mx, my
        except Exception as e:
            #print '[Info]: ', e
            return 0, 0


    def mean_token(self):
        rand = 10
        size_blob = 11
        size_x = len(self._found_x)

        #in cm
        min_x = min(self._found_x)*100
        max_x = max(self._found_x)*100
        min_y = min(self._found_y)*100
        max_y = max(self._found_y)*100
        length_x = int(round(max_x-min_x)+rand*2)
        length_y = int(round(max_y-min_y)+rand*2)
        array = np.zeros(shape=(length_y, length_x), dtype=int)
        array2 = np.zeros(shape=(length_y, length_x), dtype=int)
        for i in range(0, size_x):
            x_point = int(round(rand + self._found_x[i]*100-min_x))
            y_point = int(round(rand + self._found_y[i]*100-min_y))
            array[(y_point-size_blob):(y_point+size_blob+1), (x_point-size_blob):(x_point+size_blob+1)] = \
                array[(y_point-size_blob):(y_point+size_blob+1), (x_point-size_blob):(x_point+size_blob+1)]+1

        for i in range(0, length_x):
            for j in range(0, length_y):
                if array[j, i] <= 1:
                    array[j, i] = 0
                else:
                    array2[j, i] = 1
        s = generate_binary_structure(2, 2)
        labeled_array, num_features = label(array2, structure=s)
        # print labeled_array
        f = plt.figure(1)
        plt.imshow(array, cmap='hot', interpolation='nearest')
        plt.show()

        #print labeled_array
        f = plt.figure(1)
        plt.imshow(labeled_array, cmap='hot', interpolation='nearest')
        plt.show()

        pos_token = np.zeros(shape=(2, num_features), dtype=int)
        for i in range(0,num_features):
            positions = np.where(labeled_array == i+1)
            positions_x = positions[1]
            positions_y = positions[0]
            num = array[positions[0], positions[1]]
            num_max = max(num)
            pos_max = np.where(num >= 0.9 * num_max)
            position_x = 0.0
            position_y = 0.0
            for j in range(0, len(pos_max[0])):
                position_y = position_y+positions_y[pos_max[0][j]]
                position_x = position_x+positions_x[pos_max[0][j]]
            pos_token[0, i] = int(round(position_y/len(pos_max[0])))
            pos_token[1, i] = int(round(position_x/len(pos_max[0])))
            labeled_array[pos_token[0, i], pos_token[1, i]] = 100
        token_glob_y = (pos_token[0, :]-rand+min_y)/100
        token_glob_x = (pos_token[1, :]-rand+min_x)/100
        print 'pos_token_glob_x', token_glob_x
        print 'pos_token_glob_y', token_glob_y
        return token_glob_x, token_glob_y

        fi = plt.figure(2)
        plt.imshow(labeled_array, cmap='hot', interpolation='nearest')
        plt.show()

    def get_rotation(self, msg):
        orientation_q = msg.pose.orientation
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
    def rotate_robot(self, speed, speed_angle):
        angular_speed = speed_angle * 2 * PI / 360
        twist = Twist()
        twist.linear.x = speed
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular_speed
        t0 = rospy.Time.now().to_sec()
        self._turtlebot_pub.publish(twist)

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

def main():
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('camera_node')
    try:
        cc = CameraController()
        rospy.spin()
    except Exception as e:
        print e
        traceback.print_exc()

if __name__ == "__main__":
    main()

