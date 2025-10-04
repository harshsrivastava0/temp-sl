#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

def callbackfunc(data):
	print(euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])[2] * 180/3.141592653589)

if __name__ == '__main__':
	
	rospy.init_node('imudata', anonymous=False)
	rospy.Subscriber("/imu/data", Imu, callbackfunc)
	rospy.spin()