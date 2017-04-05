#!/usr/bin/env python

# Ros node for tbmotioncontroller for tracker trinibot with 12T tracks - assume motors have a gear ratio of 150:1

import rospy
import roslib
import actionlib
import time
roslib.load_manifest('trinibot_core')
import sys

from tbmotorcontroller import TrackedTrinibot
from trinibot_core.msg import move_trinibotAction, move_trinibotFeedback, move_trinibotResult

def process_goal_callback(goal):

    global flag
    rospy.loginfo(goal)

    if goal.objective =="goto":
        robot.drive_to_distance(goal.value)
    elif goal.objective =="turn":
        robot.turn_to_angle(goal.value)
    elif goal.objective =="spin":
        robot.turn_at_rate(goal.value)
    elif goal.objective =="speed":

        robot.drive_at_speed(goal.value)
    elif goal.objective == "stop":
        robot.stop()
    else:
        rospy.logerr("%s is not a valid objective", goal.value)
        return

    #while goal is being executed on motion controller read its feedback\\
    flag = True
    while(flag and not goal.objective == 'stop' ):
        rospy.loginfo(robot.get_feedback())
        motion_feedback = robot.get_feedback()
        if(len(motion_feedback) == 4):
            clientFeedback = move_trinibotFeedback()
            clientFeedback.xspeed = int(motion_feedback[0])
            clientFeedback.yspeed = int(motion_feedback[1])
            clientFeedback.xerror = int(motion_feedback[2])
            clientFeedback.yerror = int(motion_feedback[3])
            server.publish_feedback(clientFeedback)
        if server.is_preempt_requested():
            result = move_trinibotResult()
            result.success = False
            server.set_aborted(result, "Goal preempted")
            flag = False
            return

    #TODO we need to find way to fedback periodic distance and speed updates to the client
    result = move_trinibotResult()
    result.success = True
    server.set_succeeded(result, "Goal success")

def motor_controller_feedback(feedback):
    rospy.loginfo(feedback)

def listener():
    global robot, server
    robot = TrackedTrinibot(motor_controller_feedback, sys.argv[1])
    #Set gains to 10, 1, 1
    robot.setgains(10, 1, 1)
    rospy.init_node('tbmotioncontroller', anonymous=True)
    server = actionlib.SimpleActionServer('tbmotioncontroller', move_trinibotAction, process_goal_callback , False)
    server.start()
    rospy.spin()

if __name__ == '__main__':

    listener()
    robot.exit()

