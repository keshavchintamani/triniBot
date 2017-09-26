#!/usr/bin/env python

from sense_hat import SenseHat
import time as Time
import rospy
import tf.transformations
from pyquaternion import Quaternion
from sensor_msgs.msg import Imu, RelativeHumidity, Temperature, MagneticField
import numpy as np
from transforms3d.euler import euler2mat
from transforms3d import quaternions

D2R = 0.0174533
G = 9.81
def Start():
    sense=SenseHat()
    sense.clear()
    rospy.init_node('sense_hat', anonymous=True)
    
    #Setup the publishers
    temp_pub = rospy.Publisher("/trinibot_sensors/temperature", Temperature, queue_size= 10)
    humidity_pub = rospy.Publisher("/trinibot_sensors/humidity", RelativeHumidity, queue_size= 10)
    pose_pub = rospy.Publisher("/trinibot_sensors/imu", Imu, queue_size= 10)
    compass_pub = rospy.Publisher("/trinibot_sensors/compass", MagneticField, queue_size=10)
    
    r = rospy.Rate(50) # 10hz
    gyro = Imu()
    temp = Temperature()
    humid = RelativeHumidity()
    compass = MagneticField()
    while not rospy.is_shutdown():
        #get the gyro values
        gyro.header.stamp = rospy.Time.now()
        gyro.header.frame_id="trinibot_imu"
        #convert euler into quaternion
        quat = tf.transformations.quaternion_from_euler(sense.get_orientation_degrees()['roll']*D2R, \
                                                sense.get_orientation_degrees()['pitch']*D2R, \
                                                sense.get_orientation_degrees()['yaw']*D2R) 
        
        #quat_n90x = Quaternion(axis=[1,0,0], angle=0)#3.14159265) 
        quat= quat_NED2ENU(quat)
        #quat_enu = Quaternion(quat[0][0][0],quat[0][0][1], quat[0][0][2], quat[0][0][3])
        #quat =  quat_enu*quat_n90x
        quat_p180y = Quaternion(axis=[0, 1, 0], angle=3.14159265)
        gyro.orientation.x = quat[0][0][0]
        gyro.orientation.y = quat[0][0][1]
        gyro.orientation.z = quat[0][0][2]
        gyro.orientation.w = quat[0][0][3]

        accelerations = np.array([sense.get_accelerometer_raw()['x']*G,sense.get_accelerometer_raw()['y']*G, \
                                  sense.get_accelerometer_raw()['z']*G])
        res = quat_p180y.rotate(accelerations)
        gyro.linear_acceleration.x = res[0]
        gyro.linear_acceleration.y = -res[1]
        gyro.linear_acceleration.z = -res[2]
        angular_velocities = np.array([sense.get_gyroscope_raw()['x'], sense.get_gyroscope_raw()['y'], \
                                     sense.get_gyroscope_raw()['z']])
        res = quat_p180y.rotate(angular_velocities)
        gyro.angular_velocity.x = res[0]
        gyro.angular_velocity.y = res[1]# +res
        gyro.angular_velocity.z = res[2]# +res

        #get temperature
        temp.header.stamp = rospy.Time.now()
        temp.temperature = sense.get_temperature()
        #get humidity
        humid.header.stamp = rospy.Time.now()
        humid.relative_humidity = sense.get_humidity()
        #get compass
        compass.header.stamp = rospy.Time.now()
        compass.magnetic_field.x = sense.get_compass_raw()['x']
        compass.magnetic_field.y = sense.get_compass_raw()['y']
        compass.magnetic_field.z = sense.get_compass_raw()['z']

        temp_pub.publish(temp)
        humidity_pub.publish(humid)
        pose_pub.publish(gyro)
        compass_pub.publish(compass)
        r.sleep()

def quat_NED2ENU(iquat):
        a  = np.array([0.707, 0.707, 0, 0 ])
        b = np.array([[[iquat[3], -iquat[2], iquat[1], -iquat[0]], \
                    [iquat[2], iquat[3], -iquat[0], -iquat[1]], \
                    [-iquat[1], iquat[0], iquat[3], -iquat[2]], \
                    [iquat[0], iquat[1], iquat[2], iquat[3]]]])

        c = np.array([[[0, 0, -0.707, 0.707], \
                    [0 , 0, 0.707, 0.707], \
                    [0.707, -0.707, 0, 0], \
                    [-0.707, -0.707, 0 ,0]]])
        return (a.dot(b)).dot(c)

if __name__ == '__main__':
    Start()
