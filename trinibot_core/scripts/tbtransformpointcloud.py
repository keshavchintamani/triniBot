#!/usr/bin/env python

#This node receives a twist message and generates the appropriate action for trinibot
import time as Time
from sensor_msgs.msg import PointCloud2
import tf
import rospy
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf2_ros as tf2

tf_buffer  = tf2.Buffer()
tf_listener = tf2.TransformListener(tf_buffer, queue_size=10)

def pclReceiver(cloud_in):
    try:
        #tf_listener.waitForTransform('/lazer_frame', '/depth_frame', rospy.Time(), rospy.Duration(1))
        #transform = tf_listener.lookupTransform( '/lazer_frame', '/depth_frame', rospy.Time())
        transform = tf_buffer.lookup_transform('lazer_frame', 'base_link', rospy.Time.now())
        #print transform
        #cloud_out.header.frame_id = "lazer_frame"
        cloud_out = do_transform_cloud(cloud_in, transform)
        cloud_out.header.stamp = rospy.Time.now()
        cloud_pub.publish(cloud_out)
    except(rospy.ROSException), e:
        print "lazer frame not available, aborting..."
        print "Error message: ", e
        exit()

def init():
    global client, cloud_pub
    rospy.init_node('tbtransformpcl', anonymous=True)
    twistsub = rospy.Subscriber("/trinibot_sensors/ds525/vertex_data", PointCloud2, callback=pclReceiver)
    cloud_pub = rospy.Publisher("/rotated_cloud", PointCloud2, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    init()
