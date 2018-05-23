#!/usr/bin/env python

#This node receives a twist message and generates the appropriate action for trinibot
import time as Time
import rospy
from geometry_msgs.msg import Twist, Pose
import actionlib
import tf.transformations as tftransform
from trinibot_core.msg import  move_trinibotAction, move_trinibotGoal

def twistReceiver(twist):

    goal = move_trinibotGoal()
    velocity = twist.linear.x
    angularvelocity = twist.angular.z

    if( velocity > 0 or velocity < 0):
        goal.objective = "speed"
        goal.value = velocity
    elif(angularvelocity > 0 or angularvelocity < 0):
        goal.objective = "spin"
        goal.value = angularvelocity
    else:
        goal.objective = "stop"
        goal.value = 0

    rospy.loginfo(goal)
    client.send_goal(goal, done_cb=done_callback, feedback_cb=feedback_callback)

def poseReceiver(pose):

    goal = move_trinibotGoal()
    position = pose.position.x
    rospy.loginfo(pose)
    try:
        angle = tftransform.euler_from_quaternion(pose.orientation)
    except (TypeError):
        rospy.logwarn("Received bad quaternion")

    if (position > 0 or position < 0):
        goal.objective = "goto"
        goal.value = position
    elif (angle[2]> 0 or angle[2] < 0):
        goal.objective = "turn"
        goal.value = angle[2]
    else:
        goal.objective = "stop"
        goal.value = 0

    rospy.loginfo(goal)
    client.send_goal(goal, done_cb=done_callback, feedback_cb=feedback_callback)

def goalCancel():
    client.cancel_goal()

def feedback_callback(feedback):
    rospy.loginfo("Received result: %s", feedback)

def done_callback(result1, result2):
    rospy.loginfo("Received result: %s", result1)
    rospy.loginfo("Received result: %s", result2)

def init():
    global client
    rospy.init_node('tbactiongenerator', anonymous=True)
    twistsub = rospy.Subscriber("cmd_vel", Twist, callback=twistReceiver)
    posesub  = rospy.Subscriber("cmd_pos", Pose, callback= poseReceiver)
    # Setup the publishers
    client = actionlib.SimpleActionClient('tbmotioncontroller', move_trinibotAction)
    client.wait_for_server()
    rospy.spin()

if __name__ == '__main__':
    init()
