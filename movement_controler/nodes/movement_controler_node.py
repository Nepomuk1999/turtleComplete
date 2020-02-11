#!/usr/bin/env python

import os
import sys
import time
import traceback
import actionlib
import matplotlib.pyplot as plt
import numpy as np
from playsound import playsound
import rospy
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Int16, Int16MultiArray, String
from explore_labyrinth_srv.srv import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose

# specify directions
FRONT_LEFT = 0
FRONT_RIGHT = 3
BACK_LEFT = 2
BACK_RIGHT = 1

STAT_STOP_BOT = 'stop_bot'
STAT_MAPPING = 'mapping'
STAT_SAVE = 'save_token'
STAT_FINISH = 'finish'
STAT_ROTATE = 'rotate'
STAT_END = 'end'

PI = 3.1415926535897

if os.name == 'nt':
    pass
else:
    import termios

class MovementController:

    def __init__(self):
        print 'wait for move base'
        self._move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self._move_base_client.wait_for_server()
        print 'move base server connected'
        # self._labyrinth_explorer = rospy.Subscriber('explorer_goal_pos_result', MoveBaseGoal,
        #                                             self.labyrinth_explorer_callback)
        print 'wait for explorer service'
        self._explore_service = rospy.ServiceProxy('explorer_goal_pos', ExploreLabyrinth, headers=None)
        rospy.wait_for_service('explorer_goal_pos')
        print 'labirynth explore service connected'
        self._status = STAT_MAPPING
        self._current_goal_msg = None
        self._turtlebot_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        print 'wait for robot pose'
        msg = rospy.wait_for_message('robot_pose', Pose)
        print 'robot pose recived'
        self._start_x = msg.position.x
        self._start_y = msg.position.y
        self._free_direction = None
        self._old_free_direction = None
        self._interrupt_sub = rospy.Subscriber('interrupt_msg', String, self.interrupt_callback)
        self._interrupt_pub = rospy.Publisher('save_tags', String, queue_size=10)
        self._start_pose_pub = rospy.Publisher('start_pose', Pose, queue_size=10)
        self._last_x = 0.0
        self._last_y = 0.0

    def interrupt_callback(self, msg):
        if msg.data is STAT_MAPPING or STAT_STOP_BOT:
            self._status = msg.data
    #
    # def labyrinth_explorer_callback(self, data):
    #     self._current_goal_msg = data

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

    def stop_move_base(self):
        # print self._move_base_client.get_state()
        if self._move_base_client.get_state() is GoalStatus.ACTIVE or GoalStatus.PENDING:
            self._move_base_client.cancel_all_goals()

    def control_loop(self):
        counter = 0
        while counter < 10:
            p = Pose()
            p.position.x = self._start_x
            p.position.y = self._start_y
            self._start_pose_pub.publish(p)
        while not rospy.is_shutdown():
            try:
                response = self._explore_service(ExploreLabyrinthRequest(0, 0))
                if response.x == self._start_x and response.y == self._start_y and self._status != STAT_FINISH:
                    str = String(STAT_SAVE)
                    print str.data
                    self._interrupt_pub.publish(str)
                    self._status = STAT_FINISH
                elif self._status == STAT_ROTATE:
                    print 'start rotation'
                    self.rotate_robot(0.0, 90.0, 360.0)
                    print 'stop rotation'
                    self.stop_turtlebot()
                    self._status = STAT_MAPPING
                elif self._status == STAT_END:
                    print STAT_END
                    playsound('R2D2.mp3')
                else:
                    self._last_x = response.x
                    self._last_y = response.y
                    goal = MoveBaseGoal()
                    goal.target_pose.header.frame_id = "map"
                    goal.target_pose.header.stamp = rospy.Time.now()
                    goal.target_pose.pose.position.x = response.x
                    goal.target_pose.pose.position.y = response.y
                    goal.target_pose.pose.orientation.w = 1
                    self._current_goal_msg = goal
                    self._move_base_client.send_goal(self._current_goal_msg)
                    # self._move_base_client.wait_for_result(rospy.Duration.from_sec(40))
                    self._move_base_client.wait_for_result()
                    self.stop_move_base()
                    if self._status == STAT_FINISH:
                        self._status = STAT_END
                    else:
                        self._status = STAT_ROTATE
            except Exception as e:
                print e

    def chek_for_start_pose_in_range(self, cx, cy):
        b = False
        range = 0.1
        if self._start_x-range < cx < self._start_x + range:
            if self._start_y-range < cy < self._start_y + range:
                b = True
        return b

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
