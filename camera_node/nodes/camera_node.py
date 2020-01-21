#!/usr/bin/env python

import os
import sys
import time
import traceback
import cv2
import math
import actionlib
import matplotlib.pyplot as plt
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Int16, Int16MultiArray
from explore_labyrinth_srv.srv import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#TODO delete me

if os.name == 'nt':
    pass
else:
    import termios

class CameraController:

    def __init__(self):
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.image_callback)
        self._current_image = None
        self._binary_image = None
        rospy.wait_for_message("/image_raw", Image)
        self._odom_sub = rospy.Subscriber('/odom', Odometry, self.pose_callback)
        self._current_pose = None
        self._current_x = None
        self._current_y = None
        self._current_orientation = None
        rospy.wait_for_message('/odom', Odometry)
        self._phi = 0.0

    def control_loop(self):
        while not rospy.is_shutdown():
            rospy.wait_for_message("/image_raw", Image)
            current_binary_image = self._binary_image
            rospy.wait_for_message('/odom', Odometry)
            current_x = self._current_x
            current_y = self._current_y
            opening = self.position_token(current_binary_image, current_x, current_y)
            cv2.imshow('opening', opening)
            # cv2.waitKey(1)

    def pose_callback(self, msg):
        self._current_pose = msg.pose.pose
        self._current_x = self._current_pose.position.x
        self._current_y = self._current_pose.position.y
        self._current_orientation = self._current_pose.orientation

    def image_callback(self, msg):
        bridge = CvBridge()
        try:
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
            img_hsv = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(img_hsv, (60, 30, 30), (100, 255, 180))
            #croped = cv2.bitwise_and(cv2_img, cv2_img, mask=mask)
            kernel = np.ones((3, 3), np.float32) / 25
            self._binary_image = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        except CvBridgeError, e:
            print(e)

    def get_rotation(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self._phi = yaw

    def position_token(self, median, pos_robot_x, pos_robot_y):
        found = False
        height, width = np.shape(median)

        # find contours in the binary image
        img, contours, hierarchy = cv2.findContours(median, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # print contours
        for c in contours:
            # calculate moments for each contour
            M = cv2.moments(c)

            # calculate x,y coordinate of center
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                found = True
            else:
                cX, cY = 0, 0

            cv2.circle(img, (cX, cY), 5, (100, 100, 100), -1)
            # cv2.putText(img, "centroid", (cX - 25, cY - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            # display the image
        # cv2.imshow("Image", img)
        # cv2.waitKey(0)

        if found == True:
            middel_height = cY  # round((height_1+height_2)/2)
            middel_width = cX  # round((width_1+width_2)/2)
            print 'mw', middel_width
            print 'mh', middel_height
            point_1 = np.array([middel_width, middel_height, 1])
            H = np.array([[96.0070653929683, 308.829767885900, -14250.2707091498],
                          [19.2638464106271, 738.626135977933, -22187.7158339254],
                          [0.0143717792767875, 0.514067001440494, 1]])
            point_2 = H.dot(point_1)
            px = point_2[0] / point_2[2]
            py = point_2[1] / point_2[2]
            px_robot = 1200 / 2 - px
            py_robot = 255 + 1200 - py

            token_rob = np.array([px_robot, py_robot, 1])
            print token_rob

            # ToDO check if x and y are ok
            T = np.array([[math.cos(self._phi), -math.sin(self._phi), pos_robot_x],
                          [math.sin(self._phi), math.cos(self._phi), pos_robot_y],
                          [0, 0, 1]])
            token_glob = T.dot(token_rob)
            print 'px_robot:', pos_robot_x
            print 'py_robot:', pos_robot_y


            #        if ((middel_width-(width/2))>5):
            #            pass#direction =;
            #        elif ((middel_width-(width/2))<-5):
            #            pass#direction =;
            #        else:
            #            pass#direction

            median[middel_height, middel_width] = 0
        return median

def main():
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('camera_node')
    try:
        cc = CameraController()
        cc.control_loop()
    except Exception as e:
        print e
        traceback.print_exc()


if __name__ == "__main__":
    main()

