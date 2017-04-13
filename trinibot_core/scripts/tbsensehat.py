#!/usr/bin/env python

from sense_hat import SenseHat
import time as Time
import rospy

from sensor_msgs.msg import Imu, RelativeHumidity, Temperature, MagneticField

def Start():
    sense=SenseHat()
    rospy.init_node('sense_hat', anonymous=True)
    
    #Setup the publishers
    temp_pub = rospy.Publisher("/trinibot_sensors/temperature", Temperature, queue_size= 10)
    humidity_pub = rospy.Publisher("/trinibot_sensors/humidity", RelativeHumidity, queue_size= 10)
    pose_pub = rospy.Publisher("/trinibot_sensors/pose", Imu, queue_size= 10)
    compass_pub = rospy.Publisher("/trinibot_sensors/compass", MagneticField, queue_size=10)
    
    r = rospy.Rate(10) # 10hz
    gyro = Imu()
    temp = Temperature()
    humid = RelativeHumidity()
    compass = MagneticField()
    while not rospy.is_shutdown():
        #get the gyro values

        gyro.angular_velocity.x = sense.get_gyroscope()['pitch']
        gyro.angular_velocity.y = sense.get_gyroscope()['roll']
        gyro.angular_velocity.z = sense.get_gyroscope()['yaw']
        gyro.orientation.x = sense.get_orientation_degrees()['pitch']
        gyro.orientation.y = sense.get_orientation_degrees()['roll']
        gyro.orientation.z = sense.get_orientation_degrees()['yaw']
        gyro.linear_acceleration.x = sense.get_accelerometer_raw()['x']
        gyro.linear_acceleration.y = sense.get_accelerometer_raw()['y']
        gyro.linear_acceleration.z = sense.get_accelerometer_raw()['z']
        
        #get temperature
        temp.temperature = sense.get_temperature()
        #get humidity
        humid.relative_humidity = sense.get_humidity()
        #get compass
        compass.magnetic_field = sense.get_compass_raw()

        temp_pub.publish(temp)
        humidity_pub.publish(humid)
        pose_pub.publish(gyro)
        compass_pub.publish(compass)

        r.sleep()

if __name__ == '__main__':
    Start()
