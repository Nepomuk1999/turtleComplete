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

# specify directions
FRONT_LEFT = 0
Front_RIGHT = 1
BACK_LEFT = 2
BACK_RIGHT = 3

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

        # self._labyrinth_explorer_clint = actionlib.SimpleActionClient('/explorer_goal_pos', MoveBaseAction)
        # self._labyrinth_explorer_clint.wait_for_server()
        print 'wait for explorer service'
        rospy.wait_for_service('/explorer_goal_pos')
        self._explore_service = rospy.ServiceProxy('/explorer_goal_pos', ExploreLabyrinth, headers=None)
        print 'labirynth explore service connected'
        self._status = 'mapping'
        self._old_goal_msg = None
        self._current_goal_msg = None
        self._avoid_obstacle = rospy.Subscriber('/obstacle_avoidance', Int16, self.avoid_obstacle_callback)
        self._turtlebot_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self._odom_sub = rospy.Subscriber('/odom', Odometry, self.pose_callback)
        self._current_pose = None

    def pose_callback(self, msg):
        self._current_pose = msg.twist.twist

    def labyrinth_explorer_callback(self, data):
        self._old_goal_msg = self._current_goal_msg
        self._current_goal_msg = data

    def avoid_obstacle_callback(self, data):
        data = data.data
        print data
        if data is not -1:
            self._status = 'unwedge'
            print 'send twist'
            self.stop_bot()
            target_linear_vel = 0.0
            target_angular_vel = 0.0
            if data is FRONT_LEFT:
                target_linear_vel = 0.15
                target_angular_vel = self._current_pose.linear.x + 0.28
            if data is Front_RIGHT:
                target_linear_vel = 0.15
                target_angular_vel = self._current_pose.linear.x - 0.28
            if data is BACK_LEFT:
                target_linear_vel = 0.15
                target_angular_vel = self._current_pose.linear.x + 0.28
            if data is BACK_RIGHT:
                target_linear_vel = 0.15
                target_angular_vel = self._current_pose.linear.x - 0.28

            twist = Twist()
            print target_linear_vel
            print target_angular_vel
            twist.linear.x = target_linear_vel
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = target_angular_vel
            self._turtlebot_pub.publish(twist)
            time.sleep(3)
            self._turtlebot_pub.publish(Twist())
            #self.stop_bot()
        else:
            self._status = 'mapping'

    def stop_bot(self):
        if self._move_base_client.get_state() is GoalStatus.ACTIVE:
            self._move_base_client.cancel_all_goals()

    def control_loop(self):
        print 'movment_controler start loop'
        while not rospy.is_shutdown():
            if self._status is 'mapping':
                # self._labyrinth_explorer_clint.send_goal(MoveBaseGoal())
                # self._labyrinth_explorer_clint.wait_for_result()
                #Service update
                response = self._explore_service(ExploreLabyrinthRequest(0, 0))
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "/map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = response.x
                goal.target_pose.pose.position.y = response.y
                goal.target_pose.pose.orientation.w = 1
                self._current_goal_msg = goal
                print 'pub goal'
                self._move_base_client.send_goal(self._current_goal_msg)
                self._move_base_client.wait_for_result()



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
