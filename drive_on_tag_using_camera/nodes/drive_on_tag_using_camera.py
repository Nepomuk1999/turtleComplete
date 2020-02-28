#!/usr/bin/env python

"""
Node for Phase 2
as soon as robot stands near by a token, this Node rottes the robot and collects the exsct x, y values of the roi
send by PixyCam. Then it calculates the exact global position of the Token
"""

import math
import os
import time
import traceback

import numpy as np
import rospy
import tf
from correct_pos_srv.srv import *
from geometry_msgs.msg import Twist, PointStamped, Point, PoseWithCovarianceStamped
from pixy_msgs.msg import PixyData
from scipy.ndimage import label, generate_binary_structure
from tf.transformations import euler_from_quaternion

if os.name == 'nt':
    pass
else:
    import termios

PI = 3.1415926535897
POSE_DEVIATION = 0.90

class DriveTagCamera:

    def __init__(self):
        self._found_y = []
        self._found_x = []
        self._current_pose_time_stamp = time
        self._turtlebot_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.target_blobb_x = None
        self.target_blobb_y = None
        self._as = rospy.Service('drive_on_tag', CorrectPosSrv, self.drive_callback)
        self._current_pos_x = None
        self._current_pos_y = None
        self._orientation = None
        self._pose_pub_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        print 'wait for pose'
        rospy.wait_for_message('amcl_pose', PoseWithCovarianceStamped)
        print 'got pose'
        self.blob_sub = rospy.Subscriber('block_data', PixyData, self.blobb_callback)
        self._tl = tf.TransformListener()
        print 'init finish'

    def drive_callback(self, data):
        print 'got request'
        self.target_blobb_x = data.searched_tag_x
        self.target_blobb_y = data.searched_tag_y
        rospy.wait_for_message('amcl_pose', PoseWithCovarianceStamped)
        self.find_blob()
        self.stop_turtlebot()
        rospy.sleep(0.01)
        while len(self._found_y) < 10:
            self.move_straight(0.10)
            self.stop_turtlebot()
            self.rotate_robot(0.0, 20, 20)
            time.sleep(0.5)
        print 'found 5 blobbs'
        fx, fy = self.mean_token()
        print 'fx: ', fx
        print 'fy: ', fy
        if self.target_is_current(fx, fy, self.target_blobb_x, self.target_blobb_y):
            res = CorrectPosSrvResponse()
            res.correct_x = fx
            res.correct_y = fy
            return res
        else:
            res = CorrectPosSrvResponse()
            res.correct_x = -100.0
            res.correct_y = -100.0
            return res

    def target_is_current(self, cx, cy, tx, ty):
        b = False
        if cx and cy != 0.0:
            xu = tx + POSE_DEVIATION / 2
            xl = tx - POSE_DEVIATION / 2
            yu = ty + POSE_DEVIATION / 2
            yl = ty - POSE_DEVIATION / 2
            if xl <= cx <= xu:
                if yl <= cy <= yu:
                    b = True
        return b

    def mean_token(self):
        rand = 10
        size_blob = 10
        size_x = len(self._found_x)

        # in cm
        min_x = min(self._found_x) * 100
        max_x = max(self._found_x) * 100
        min_y = min(self._found_y) * 100
        max_y = max(self._found_y) * 100
        length_x = int(round(max_x - min_x) + rand * 2)
        length_y = int(round(max_y - min_y) + rand * 2)
        array = np.zeros(shape=(length_y, length_x), dtype=int)
        array2 = np.zeros(shape=(length_y, length_x), dtype=int)
        for i in range(0, size_x):
            x_point = int(round(rand + self._found_x[i] * 100 - min_x))
            y_point = int(round(rand + self._found_y[i] * 100 - min_y))
            array[(y_point - size_blob):(y_point + size_blob + 1), (x_point - size_blob):(x_point + size_blob + 1)] = \
                array[(y_point - size_blob):(y_point + size_blob + 1),
                (x_point - size_blob):(x_point + size_blob + 1)] + 1

        for i in range(0, length_x):
            for j in range(0, length_y):
                if array[j, i] <= 1:
                    array[j, i] = 0
                else:
                    array2[j, i] = 1
        s = generate_binary_structure(2, 2)
        labeled_array, num_features = label(array2, structure=s)
        pos_token = np.zeros(shape=(2, num_features), dtype=int)
        for i in range(0, num_features):
            positions = np.where(labeled_array == i + 1)
            positions_x = positions[1]
            positions_y = positions[0]
            num = array[positions[0], positions[1]]
            num_max = max(num)
            pos_max = np.where(num >= 0.7 * num_max)
            position_x = 0.0
            position_y = 0.0
            for j in range(0, len(pos_max[0])):
                position_y = position_y + positions_y[pos_max[0][j]]
                position_x = position_x + positions_x[pos_max[0][j]]
            pos_token[0, i] = int(round(position_y / len(pos_max[0])))
            pos_token[1, i] = int(round(position_x / len(pos_max[0])))
            labeled_array[pos_token[0, i], pos_token[1, i]] = 100
        token_glob_y = (pos_token[0, :] - rand + min_y) / 100
        token_glob_x = (pos_token[1, :] - rand + min_x) / 100
        print 'pos_token_glob_x', token_glob_x
        print 'pos_token_glob_y', token_glob_y
        return token_glob_x, token_glob_y

    def get_pose_token_robot_coord(self, blob_x, blob_y):
        middel_height = blob_y
        middel_width = blob_x
        point_1 = np.array([middel_height, middel_width, 1])
        H = np.array([[4.60672455146226, 9.72835892564822, -28.9881784213841],
                      [1.36970419271895, 21.7962377297899, 97.6048947143528],
                      [0.00129456445901604, 0.0167026450466100, 1]])
        point_2 = H.dot(point_1)
        py = point_2[0] / point_2[2]
        px = point_2[1] / point_2[2]
        py_robot = (1200 / 2 - py) / 1000
        px_robot = (1200 - px) / 1000
        return px_robot, py_robot

    def blobb_callback(self, blob_data):
        stamp_nsec = blob_data.header.stamp.nsecs
        if stamp_nsec != 0:
            self._last_stamp = blob_data.header.stamp
            found_y = blob_data.blocks[0].roi.x_offset
            found_x = blob_data.blocks[0].roi.y_offset
            rc_blob_x, rc_blob_y = self.get_pose_token_robot_coord(found_x, found_y)
            mc_blob_x, mc_blob_y = self.get_pose_token_map(rc_blob_x, rc_blob_y, self._last_stamp)
            if mc_blob_y != 0 and mc_blob_x != 0:
                print 'mc_blob_x:', mc_blob_x
                print 'mc_blob_y:', mc_blob_y
                self._found_x.append(mc_blob_x)
                self._found_y.append(mc_blob_y)

    def pose_callback(self, msg):
        msg = msg.pose
        self._current_pose_time_stamp = time.time()
        self._orientation = msg.pose.orientation
        self._current_pos_x = msg.pose.position.x
        self._current_pos_y = msg.pose.position.y

    def find_blob(self):
        self._found_y = []
        self._found_x = []
        print 'cx: ', self._current_pos_x
        print 'cy: ', self._current_pos_y
        print 'bx: ', self.target_blobb_x
        print 'by: ', self.target_blobb_y
        t_yaw = math.atan2(self.target_blobb_y - self._current_pos_y, self.target_blobb_x - self._current_pos_x)
        r_yaw = self.get_rotation()
        diff = t_yaw - r_yaw
        diff = diff/PI * 180
        speed = 15.0
        print 't_yaw: ', t_yaw
        print 'r_yaw: ', r_yaw
        if diff < 0:
            speed = -speed
        diff = abs(diff)
        print 'start rotate'
        b = self.rotate_robot(0.0, speed, diff)
        self.stop_turtlebot()
        print 'stop rotate'
        self._found_y = []
        self._found_x = []

    def get_rotation(self):
        orientation_q = self._orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        return yaw


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
        while current_angle <= relative_angle:
            # print 'cangle: ', current_angle
            # print 'rel angle: ', current_angle
            t1 = rospy.Time.now().to_sec()
            current_angle = abs(angular_speed) * (t1 - t0)
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


def main():
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('movement_controler')
    try:
        mc = DriveTagCamera()
        rospy.spin()
    except Exception as e:
        print e
        traceback.print_exc()


if __name__ == "__main__":
    main()
