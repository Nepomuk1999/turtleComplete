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
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from map_tag_handler_srv.srv import *


# specify directions
EVERYWHERE = 0
FRONT_LEFT = 1
FRONT_RIGHT = 4
BACK_LEFT = 3
BACK_RIGHT = 2

PI = 3.1415926535897
VEL_STRAIGHT = 0.15
TIME_STRAIGHT = 2

if os.name == 'nt':
    pass
else:
    import termios

class MovementController:

    def __init__(self):
        self._move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self._move_base_client.wait_for_server()
        print 'move base server connected'
        self._labyrinth_explorer = rospy.Subscriber('/explorer_goal_pos_result', MoveBaseGoal,
                                                    self.labyrinth_explorer_callback)
        self._tag_service = rospy.ServiceProxy('/get_next_Tag', TagService, headers=None)
        print 'wait for explorer service'
        rospy.wait_for_service('/explorer_goal_pos')
        self._explore_service = rospy.ServiceProxy('/explorer_goal_pos', ExploreLabyrinth, headers=None)
        print 'labirynth explore service connected'
        self._status = 'find_pos'
        self._old_goal_msg = None
        self._current_goal_msg = None
        self._avoid_obstacle = rospy.Subscriber('/obstacle_avoidance', Int16, self.avoid_obstacle_callback)
        self._turtlebot_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self._odom_sub = rospy.Subscriber('/odom', Odometry, self.pose_callback)
        self._current_pose = None
        self._free_direction = None
        self._old_free_direction = None

    def pose_callback(self, msg):
        self._current_pose = msg.twist.twist
        print 'pose callback', self._current_pose

    def labyrinth_explorer_callback(self, data):
        self._old_goal_msg = self._current_goal_msg
        self._current_goal_msg = data

    def avoid_obstacle_callback(self, data):
        data = data.data
        self._free_direction = data
        if data is not -1 and self._old_free_direction == -1:
            self._old_free_direction = data
            temp = self._status
            self._status = 'unwedge'
            print 'send twist'
            self.stop_move_base()
            if data is FRONT_LEFT:
                self.rotate_robot(0.15, -15.0, 45.0)
            if data is FRONT_RIGHT:
                self.rotate_robot(0.15, 15.0, 45.0)
            if data is BACK_LEFT:
                self.rotate_robot(-0.15, -15.0, 45.0)
            if data is BACK_RIGHT:
                self.rotate_robot(-0.15, 15.0, 45.0)
            self.stop_turtlebot()
            self._status = temp
        else:
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
        current_angle = 0
        while current_angle < relative_angle:
            self._turtlebot_pub.publish(twist)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed * (t1 - t0)
        self.stop_turtlebot()

    def stop_turtlebot(self):
        self._turtlebot_pub.publish(Twist())

    def stop_move_base(self):
        # print self._move_base_client.get_state()
        if self._move_base_client.get_state() is GoalStatus.ACTIVE or GoalStatus.PENDING:
            self._move_base_client.cancel_all_goals()

    def control_loop(self):
        print 'movment_controler start loop'
        # rotate at start for better map
        # self.rotate_robot()
        last_x = -10.0
        last_y = -10.0
        while not rospy.is_shutdown():
            if self._status is 'pose_estimation':
                self.rotate_robot(0.0, 45, 359)
                self.stop_turtlebot()
                self.move_straight(VEL_STRAIGHT)
                t0 = rospy.Time.now().to_sec() + TIME_STRAIGHT
                while self._free_direction is EVERYWHERE and t0 < rospy.Time.now().to_sec()
                    pass
                self.stop_turtlebot()
                #pose est abfrage
            elif self._status is 'find_tsgs':
                response = self._tag_service(TagServiceRequest)
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "/map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = response.x
                goal.target_pose.pose.position.y = response.y
                goal.target_pose.pose.orientation.w = 1
                self._current_goal_msg = goal
                print 'pub goal'
                print response.x
                print response.y
                self._move_base_client.send_goal(self._current_goal_msg)
                # self._move_base_client.wait_for_result(rospy.Duration.from_sec(20))
                self._move_base_client.wait_for_result(rospy.Duration.from_sec(60))



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
