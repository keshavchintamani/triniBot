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

    while not rospy.is_shutdown():
        value = Imu()
        value.angular_velocity.x=sense.get_gyroscope()
        pose_pub.publish()
        Time.sleep(1)


if __name__ == '__main__':
    Start()

