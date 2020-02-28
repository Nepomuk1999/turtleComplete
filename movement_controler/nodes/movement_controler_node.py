#!/usr/bin/env python

"""
Node implements the control for Phase 1 of the Project, communicates with the Exploration Node, which chooses the next
unknown pose as goal, and sends the goal to move_base.
It publishes a msg in the end to tell the camera_node to calculate the positions of the tokens and send them to the
map_tag_handler
"""

import os
import traceback

import actionlib
# from playsound import playsound
import rospy
from actionlib_msgs.msg import GoalStatus
from explore_labyrinth_srv.srv import *
from geometry_msgs.msg import Twist, Pose, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String

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
        print 'wait for explorer service'
        self._explore_service = rospy.ServiceProxy('explorer_goal_pos', ExploreLabyrinth, headers=None)
        rospy.wait_for_service('explorer_goal_pos')
        print 'labirynth explore service connected'
        self._status = STAT_ROTATE
        self._current_goal_msg = None
        self._turtlebot_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        print 'wait for robot pose'
        # Publisher for another pose node
        # msg = rospy.wait_for_message('pose', PoseStamped)
        # print 'robot pose recived'
        # self._start_x = msg.pose.position.x
        # self._start_y = msg.pose.position.y

        msg = rospy.wait_for_message('robot_pose', Pose)
        print 'robot pose recived'
        self._start_x = msg.position.x
        self._start_y = msg.position.y

        self._free_direction = None
        self._old_free_direction = None
        self._interrupt_sub = rospy.Subscriber('interrupt_msg', String, self.interrupt_callback)
        self._interrupt_pub = rospy.Publisher('save_tags', String, queue_size=10)
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
        self._turtlebot_pub.publish(twist)
        while current_angle < relative_angle:
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

    def stop_move_base(self):
        # print self._move_base_client.get_state()
        if self._move_base_client.get_state() is GoalStatus.ACTIVE or GoalStatus.PENDING:
            self._move_base_client.cancel_all_goals()

    def control_loop(self):
        calculate = True
        rotate_left = True
        while not rospy.is_shutdown():
            print 'status = ', self._status
            try:
                if calculate:
                    # get next goal from explore node
                    req = ExploreLabyrinthRequest()
                    req.x = self._start_x
                    req.y = self._start_y
                    response = self._explore_service(req)
                    calculate = False
                if self.chek_for_start_pose_in_range(response.x, response.y) and self._status != STAT_FINISH:
                    str = String(STAT_SAVE)
                    print str.data
                    self._interrupt_pub.publish(str)
                    print 'stat = finish'
                    self._status = STAT_FINISH
                elif self._status == STAT_ROTATE:
                    print 'start rotation'
                    if rotate_left:
                        b = self.rotate_robot(0.0, 30.0, 360.0)
                        rotate_left = False
                    else:
                        b = self.rotate_robot(0.0, -30.0, 360.0)
                        rotate_left = True
                    print 'stop rotation'
                    self.stop_turtlebot()
                    self._status = STAT_MAPPING
                elif self._status == STAT_END:
                    print STAT_END
                    #playsound('/home/christoph/catkin_ws/src/movement_controler/nodes/R2D2.mp3')
                else:
                    self._last_x = response.x
                    self._last_y = response.y
                    goal = MoveBaseGoal()
                    goal.target_pose.header.frame_id = "bauwen/map"
                    goal.target_pose.header.stamp = rospy.Time.now()
                    goal.target_pose.pose.position.x = response.x
                    goal.target_pose.pose.position.y = response.y
                    goal.target_pose.pose.orientation.w = 1
                    self._current_goal_msg = goal
                    self._move_base_client.send_goal(self._current_goal_msg)
                    # self._move_base_client.wait_for_result(rospy.Duration.from_sec(40))
                    self._move_base_client.wait_for_result()
                    #if self._move_base_client.get_state() is GoalStatus.SUCCEEDED:
                    calculate = True
                    self.stop_move_base()
                    if self._status == STAT_FINISH:
                        self._status = STAT_END
                    elif self._status == STAT_MAPPING:
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
