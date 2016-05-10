#!/usr/bin/env python

from sense_hat import SenseHat
import time as Time
import rospy

from sensor_msgs.msg import Imu, RelativeHumidity, Temperature, MagneticField

def Start():
    sense=SenseHat()
    rospy.init_node('sense_hat', anonymous=True)
    #TODO
    #temp_pub = rospy.Publisher("/sensehat/temperature", Temperature, queue_size= 10)
    #humidity_pub = rospy.Publisher("/sensehat/humidity", RelativeHumidity, queue_size= 10)
    pose_pub = rospy.Publisher("/sensehat/pose", Imu, queue_size= 10)
    compass_pub = rospy.Publisher("/sensehat/compass", MagneticField, queue_size=10)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        gyro = Imu()
        gyro.angular_velocity.x = sense.get_gyroscope()['pitch']
        gyro.angular_velocity.y = sense.get_gyroscope()['roll']
        gyro.angular_velocity.z = sense.get_gyroscope()['yaw']
        gyro.orientation.x = sense.get_orientation_degrees()['pitch']
        gyro.orientation.y = sense.get_orientation_degrees()['roll']
        gyro.orientation.z = sense.get_orientation_degrees()['yaw']
        gyro.linear_acceleration.x = sense.get_accelerometer_raw()['x']
        gyro.linear_acceleration.y = sense.get_accelerometer_raw()['y']
        gyro.linear_acceleration.z = sense.get_accelerometer_raw()['z']
        #rospy.loginfo("Sending IMU data:%s", gyro)
        pose_pub.publish(gyro)

        compass= MagneticField()
        # magnetic intensity of the axis in microteslas
        compass.magnetic_field.x = sense.get_compass_raw()['x']
        compass.magnetic_field.y = sense.get_compass_raw()['y']
        compass.magnetic_field.z = sense.get_compass_raw()['z']
        compass_pub.publish(compass)
        #rospy.loginfo("Sending magnetic data:%s", compass)
        r.sleep()

if __name__ == '__main__':
    Start()

