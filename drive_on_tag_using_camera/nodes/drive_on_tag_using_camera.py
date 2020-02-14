#!/usr/bin/env python

import os
import sys
import traceback
import rospy
from explore_labyrinth_srv.srv import *
from geometry_msgs.msg import Twist, Pose, PointStamped, Point
from pixy_msgs.msg import PixyData
from rospy import Time

if os.name == 'nt':
    pass
else:
    import termios

class DriveTagCamera:

    def __init__(self):
        self._turtlebot_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self._as = rospy.Service('drive_on_tag', ExploreLabyrinth, self.drive_callback)
        self._pose_pub_sub = rospy.Subscriber('pose', Pose, self.pose_callback)
        self._current_pos_x = None
        self._current_pos_y = None
        self.blob_sub = rospy.Subscriber('block_data', PixyData, self.blobb_callback)
        self._blob_y = 0.0
        self._blob_x = 0.0
        self._last_stamp = Time()

    def drive_callback(self, data):
        target_blobb_x = data.x
        target_blobb_y = data.y
        rospy.wait_for_message('pose', Pose)
        b = self.find_blobb()


    def blobb_callback(self, blob_data):
        stamp_nsec = blob_data.header.stamp.nsecs
        if stamp_nsec != 0:
            self._last_stamp = blob_data.header.stamp
            self._blob_y = blob_data.blocks[0].roi.x_offset
            self._blob_x = blob_data.blocks[0].roi.y_offset

    def pose_callback(self, data):
        self._current_pos_x = data.position.x
        self._current_pos_y = data.position.y

    def find_blob(self):
        pass

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
        while current_angle < relative_angle:
            #TODO check if twist has to be sendt
            #self._turtlebot_pub.publish(twist)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed * (t1 - t0)
        self.stop_turtlebot()

    def stop_turtlebot(self):
        self._turtlebot_pub.publish(Twist())



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
