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
import tf
#import scipy.ndimage
from scipy.ndimage import label, generate_binary_structure
from PIL import Image as Im
from cv_bridge import CvBridge, CvBridgeError
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Int16, Int16MultiArray
from explore_labyrinth_srv.srv import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, PointStamped, Point
from sensor_msgs.msg import Image
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import String, Time
from map_tag_handler_srv.srv import *
from save_tag_msg.msg import *
from pixy_msgs.msg import *


STAT_STOP_BOT = 'stop_bot'
STAT_MAPPING = 'mapping'
STAT_SAVE = 'save_token'

ELEMENT_RANGE_WITH = 0.5

if os.name == 'nt':
    pass
else:
    import termios

class CameraController:

    def __init__(self):
        self._pos_token_glob_x = []
        self._pos_token_glob_y = []
        self.blob_sub = rospy.Subscriber('block_data', PixyData, self.blobb_callback)
        self._blob_y = 0.0
        self._blob_x = 0.0
        self._last_stamp = Time()
        #self._current_image = None
        #self._binary_image = None
        #rospy.wait_for_message("/image_raw", Image)
        # self._odom_sub = rospy.Subscriber('/odom', Odometry, self.pose_callback)
        # self._current_pose = None
        # self._current_x = None
        # self._current_y = None
        # self._current_orientation = None
        # self._phi = None
        # rospy.wait_for_message('/odom', Odometry)
        self._pose_pub_sub = rospy.Subscriber('robot_pose', Pose, self.pose_pub_callback)
        self._current_pose_pub = None
        self._current_x_pub = None
        self._current_y_pub = None
        self._current_orientation_pub = None
        self._phi_pub = None
        print 'wait pose'
        rospy.wait_for_message('robot_pose', Pose)
        print 'got pose'
        self._found_x = []
        self._found_y = []
        self._interrupt_pub = rospy.Publisher('interrupt_msg', String, queue_size=10)
        self._interrupt_pub = rospy.Subscriber('save_tags', String, self.interrupt_callback)
        self._save_tags_pub = rospy.Publisher('save_tags_to_file', SaveTag, queue_size=10)
        msg = rospy.wait_for_message("map", OccupancyGrid)
        meta_data = msg.info
        self._offset_x = meta_data.origin.position.x
        self._offset_y = meta_data.origin.position.y
        self._resolution = meta_data.resolution
        self._tl = tf.TransformListener()
        print 'init finished'

    def blobb_callback(self, blob_data):
        stamp_nsec = blob_data.header.stamp.nsecs
        if stamp_nsec != 0:
            self._last_stamp = blob_data.header.stamp
            self._blob_y = blob_data.blocks[0].roi.x_offset
            self._blob_x = blob_data.blocks[0].roi.y_offset
            rc_blob_x, rc_blob_y = self.get_pose_token_robot_coord(self._blob_x, self._blob_y)
            mc_blob_x, mc_blob_y = self.get_pose_token_map(rc_blob_x, rc_blob_y, self._last_stamp)
            if mc_blob_y != 0 and mc_blob_x != 0:
                print 'mc_blob_x:', mc_blob_x
                print 'mc_blob_y:', mc_blob_y
                self._found_x.append(mc_blob_x)
                self._found_y.append(mc_blob_y)

    def interrupt_callback(self, msg):
        print msg
        print msg.data
        if msg.data == STAT_SAVE:
            print 'in if'
            self.mean_token()
            print 'prepare send tag to save'
            msg = SaveTag()
            msg.x_values = self._pos_token_glob_x
            msg.y_values = self._pos_token_glob_y
            self._save_tags_pub.publish(msg)
            print 'lists to save sendt'


    def pose_pub_callback(self, msg):
        self._current_pose_pub = msg
        self._current_x_pub = self._current_pose_pub.position.x
        self._current_y_pub = self._current_pose_pub.position.x
        self._current_orientation_pub = self._current_pose_pub.orientation
        self._phi_pub = self.get_rotation(msg)

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
        middel_height = blob_y  # round((height_1+height_2)/2)
        middel_width = blob_x  # round((width_1+width_2)/2)
        #print 'mw', middel_width
        #print 'mh', middel_height
        point_1 = np.array([middel_height, middel_width, 1])
        H = np.array([[9.27998422577072, 19.6251228932382, -320.157395862157 ],
                      [5.24077929985246, 38.6451497322568, -201.136898122784],
                      [0.00619017665655573, 0.0317700847033603, 1]])
        # H = np.array([[96.0070653929683, 308.829767885900, -14250.2707091498],
        #               [19.2638464106271, 738.626135977933, -22187.7158339254],
        #               [0.0143717792767875, 0.514067001440494, 1]])
        point_2 = H.dot(point_1)
        py = point_2[0] / point_2[2]
        px = point_2[1] / point_2[2]
        py_robot = (1200 / 2 - py) / 1000
        px_robot = (1200 - px) / 1000
        return px_robot, py_robot

    def get_pose_token_map(self, rc_blob_x, rc_blob_y, blob_data_stamp):
        ps = PointStamped()
        ps.header.frame_id = 'base_footprint'
        ps.header.stamp = blob_data_stamp
        #cx, cy = self.transform_to_meter(rc_blob_x, rc_blob_y)
        #ps.point = Point(x=cx, y=cy)
        ps.point = Point(x=rc_blob_x, y=rc_blob_y)
        try:
            ps_map = self._tl.transformPoint('map', ps)
            mx = ps_map.point.x
            my = ps_map.point.y
            #print 'mx', mx
            #print 'my', my
            #Return in m
            return mx, my
        except Exception as e:
            #print '[Info]: ', e
            return 0, 0




    # def pose_callback(self, msg):
    #     self._current_pose = msg.pose.pose
    #     self._current_x = self._current_pose.position.x
    #     self._current_y = self._current_pose.position.y
    #     self._current_orientation = self._current_pose.orientation
    #     self._phi_pub = self.get_rotation(msg)

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
        #print 'length_x', length_x
        #print 'length_y', length_y
        array = np.zeros(shape=(length_y, length_x), dtype=int)
        array2 = np.zeros(shape=(length_y, length_x), dtype=int)
        for i in range(0, size_x):
            x_point = int(round(rand + self._found_x[i]*100-min_x))
            y_point = int(round(rand + self._found_y[i]*100-min_y))
            #print 'x', x_point
            #print 'y', y_point
            array[(y_point-size_blob):(y_point+size_blob+1), (x_point-size_blob):(x_point+size_blob+1)] = \
                array[(y_point-size_blob):(y_point+size_blob+1), (x_point-size_blob):(x_point+size_blob+1)]+1
            #plt.imshow(array, cmap='hot', interpolation='nearest')
            #plt.show()

        #w, h = length_x, length_y
        #data = np.zeros((h, w, 3), dtype=np.uint8)

        for i in range(0, length_x):
            for j in range(0, length_y):
                if array[j, i] <= 1:
                    array[j, i] = 0
                else:
                    array2[j, i] = 1
                    #data[j, i] = 255
        s = generate_binary_structure(2, 2)
        labeled_array, num_features = label(array2, structure=s)
        #print labeled_array
        f = plt.figure(1)
        plt.imshow(labeled_array, cmap='hot', interpolation='nearest')
        plt.show()
        pos_token = np.zeros(shape=(2, num_features), dtype=int)
        pos_token_glob = np.zeros(shape=(2, num_features))
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
        self.pos_token_glob_y = (pos_token[0,:]-rand+min_y)/100
        self.pos_token_glob_x = (pos_token[1,:]-rand+min_x)/100
        print 'pos_token_glob_x', self.pos_token_glob_x
        print 'pos_token_glob_y', self.pos_token_glob_y
        # fi = plt.figure(2)
        # plti.imshow(labeled_array, cmap='hot', interpolation='nearest')
        # fi.show()

    def get_rotation(self, msg):
        orientation_q = msg.orientation
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

    def position_token(self, blob_y, blob_x, pos_robot_x, pos_robot_y, pos_robot_phi):
        # found = False
        '''
        height, width = np.shape(median)
        # find contours in the binary image
        img, contours, hierarchy = cv2.findContours(median, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        output = median.copy()

        # #Version 1
        # # print contours
        # for c in contours:
        #     # calculate moments for each contour
        #     M = cv2.moments(c)
        #
        #     # calculate x,y coordinate of center
        #     if M["m00"] != 0:
        #         cX = int(M["m10"] / M["m00"])
        #         cY = int(M["m01"] / M["m00"])
        #         found = True
        #     else:
        #         cX, cY = 0, 0
        #
        #     cv2.circle(img, (cX, cY), 5, (100, 100, 100), -1)
        #     # cv2.putText(img, "centroid", (cX - 25, cY - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        # ###
        # cv2.imshow("Image", img)

        #Version 2
        if len(contours) != 0:
            # the contours are drawn here
            #cv2.drawContours(output, contours, -1, 155, 3)

            # find the biggest area of the contour
            c = max(contours, key=cv2.contourArea)
            # calculate moments for each contour
            M = cv2.moments(c)

            # calculate x,y coordinate of center
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                found = True
            else:
                cX, cY = 0, 0

            cv2.circle(output, (cX, cY), 5, (100, 100, 100), -1)
            #found = True
            #x, y, w, h = cv2.boundingRect(c)
            # draw the 'human' contour (in green)
            #cv2.rectangle(output, (x, y), (x + w, y + h), (0, 155, 0), 2)

            # display the image
            cv2.imshow("Image2", output)

            cv2.waitKey(1)
        '''
        middel_height = blob_y  # round((height_1+height_2)/2)
        middel_width = blob_x  # round((width_1+width_2)/2)
        print 'mw', middel_width
        print 'mh', middel_height
        point_1 = np.array([middel_width, middel_height, 1])
        H = np.array([[96.0070653929683, 308.829767885900, -14250.2707091498],
                      [19.2638464106271, 738.626135977933, -22187.7158339254],
                      [0.0143717792767875, 0.514067001440494, 1]])
        point_2 = H.dot(point_1)
        py = point_2[0] / point_2[2]
        px = point_2[1] / point_2[2]
        py_robot = (1200 / 2 - py)/1000
        px_robot = (255 + 1200 - px)/1000

        token_rob = np.array([px_robot, py_robot, 1])
        #print 'token_rob', token_rob

        # ToDO check if x and y are ok
        T = np.array([[math.cos(pos_robot_phi), -math.sin(pos_robot_phi), pos_robot_x],
                      [math.sin(pos_robot_phi), math.cos(pos_robot_phi), pos_robot_y],
                      [0, 0, 1]])
        token_glob = T.dot(token_rob)
        self._found_x = np.append(self._found_x,token_glob[0])
        self._found_y = np.append(self._found_y,token_glob[1])
        #print 'len', self._found_x
        if len(self._found_x)%200== 0:
            self.mean_token()
            plt.plot(self._found_x, self._found_y, 'ro')
            plt.show()#block=False)
            #print 'phi: ', pos_robot_phi
            #print 'token_glob', token_glob
            #print 'px_robot:', pos_robot_x
            #print 'py_robot:', pos_robot_y


            #        if ((middel_width-(width/2))>5):
            #            pass#direction =;
            #        elif ((middel_width-(width/2))<-5):
            #            pass#direction =;
            #        else:
            #            pass#direction

        #   median[middel_height, middel_width] = 0
        #return median

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

