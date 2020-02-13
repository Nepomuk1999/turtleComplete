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
from geometry_msgs.msg import Twist, Pose, PoseWithCovarianceStamped
from map_tag_handler_srv.srv import *
from sensor_msgs.msg import LaserScan

# specify directions
EVERYWHERE = 0
FRONT_LEFT = 1
FRONT_RIGHT = 4
BACK_LEFT = 3
BACK_RIGHT = 2

PI = 3.1415926535897
VEL_STRAIGHT = 0.15
TIME_STRAIGHT = 2
POSE_DEVIATION = 0.5

STAT_FIND_POS = 'find_pos'
STAT_COLLECT_TAGS = 'collect_tags'

if os.name == 'nt':
    pass
else:
    import termios

class MovementController:

    def __init__(self):
        self._current_goal_x = 0.0
        self._current_goal_y = 0.0
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
        print 'movment_controler start loop'
        first_run = True
        while not rospy.is_shutdown():
            print 'calc elips'
            ea = self.calc_elips_area(self._covariance)
            print ea
            if 0.3 > abs(ea):
                self._status = STAT_COLLECT_TAGS
            else:
                self._status = STAT_FIND_POS
            print self._status
            if self._status == STAT_FIND_POS:
                #rotate
                # print 'start rotation'
                # self.rotate_robot(0.0, 50.0, 360.0)
                # print 'stop rotation'
                # self.stop_turtlebot()
                # time.sleep(1.0)
                #drive free direction
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
                time.sleep(2.0)
                self.stop_turtlebot()
                time.sleep(1.0)
            elif self._status == STAT_COLLECT_TAGS:
                if first_run or self.is_current_pos_goal_pos():
                    req = TagServiceRequest()
                    req.current_pose_x = self._current_pos_x
                    req.current_pose_y = self._current_pos_y
                    response = self._tag_service(req)
                    print'response: ', response
                    self._current_goal_x = response.tags_x
                    self._current_goal_y = response.tags_y
                    first_run = False
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = self._current_goal_x
                goal.target_pose.pose.position.y = self._current_goal_y
                goal.target_pose.pose.orientation.w = 1
                self._current_goal_msg = goal
                print 'pub goal'
                print self._current_goal_x
                print self._current_goal_y
                self._move_base_client.send_goal(self._current_goal_msg)
                # self._move_base_client.wait_for_result(rospy.Duration.from_sec(20))
                self._move_base_client.wait_for_result(rospy.Duration.from_sec(60))

    def is_current_pos_goal_pos(self):
        b = False
        if self._current_goal_x and self._current_goal_y != 0.0:
            xu = self._current_goal_x + POSE_DEVIATION / 2
            xl = self._current_goal_x - POSE_DEVIATION / 2
            yu = self._current_goal_y + POSE_DEVIATION / 2
            yl = self._current_goal_y - POSE_DEVIATION / 2
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
