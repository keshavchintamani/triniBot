#!/usr/bin/env python
from sense_hat import SenseHat
import time as Time
import rospy

from sensor_msgs.msg import Imu, RelativeHumidity, Temperature

def Start():
    sense=SenseHat()
    rospy.init_node('sense_hat', anonymous=True)
    #TODO
    #temp_pub = rospy.Publisher("/sensehat/temperature", Temperature, queue_size= 10)
    #humidity_pub = rospy.Publisher("/sensehat/humidity", RelativeHumidity, queue_size= 10)
    pose_pub = rospy.Publisher("/sensehat/pose", Imu, queue_size= 10)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        value = Imu()
        value.angular_velocity.x = sense.get_gyroscope()['pitch']
        value.angular_velocity.y = sense.get_gyroscope()['roll']
        value.angular_velocity.z = sense.get_gyroscope()['yaw']
        value.orientation.x = sense.get_orientation_radians()['pitch']
        value.orientation.y = sense.get_orientation_radians()['roll']
        value.orientation.z = sense.get_orientation_radians()['yaw']
        value.linear_acceleration.x = sense.get_accelerometer_raw()['x']
        value.linear_acceleration.y = sense.get_accelerometer_raw()['y']
        value.linear_acceleration.z = sense.get_accelerometer_raw()['z']
	rospy.loginfo("Sending IMU data:%s", value)
        pose_pub.publish(value)
        r.sleep()

if __name__ == '__main__':
    Start()

