#!/usr/bin/env python3
import math
import rospy
from race.msg import poseheader
from race.msg import tfheader
import message_filters
from ackermann_msgs.msg import AckermannDrive
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Header
from tf.transformations import euler_from_quaternion

cur_vel = 0
cur_del = 0
prevVar = [0, 0, 0]
flag = [0]


vel_input = 0.5
acc = 5
delvel = 0.1
angle = math.radians(0)

command_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size = 1)

def withinthresh(x, thresh):
    if(x>-thresh and x<thresh):return True
    return False

def control(tfdata, posedata):
	global cur_vel, prevVar, flag
	#------------time-manage-------------
	if(prevVar[0]==0.0):
		prevVar[0] = rospy.get_time()
		return

	time = rospy.get_time()-prevVar[0]
	if(withinthresh(time, 0.3)):
		return
	prevVar[0] = rospy.get_time()


	cur_del = (euler_from_quaternion([tfdata.transform.rotation.x, tfdata.transform.rotation.y, tfdata.transform.rotation.z, tfdata.transform.rotation.w])[2])
    
	x = posedata.pose.position.x
	y = posedata.pose.position.y

	if(flag[0]): cur_vel = (((x-prevVar[1])/time)**2 + ((y-prevVar[2])/time)**2)**0.5
	flag[0] = 1
	prevVar[1] = x
	prevVar[2] = y


	command = AckermannDrive()
	# command.speed = cur_vel +  acc*time
	# command.steering_angle = cur_del + delvel*time
	command.speed = vel_input
	# command.steering_angle = math.radians(-2)
	

	msgtosend = AckermannDriveStamped()
	msgtosend.header = Header()
	msgtosend.header.stamp = rospy.Time.now()
	msgtosend.drive = command

	# print("cur vel: ", cur_vel)
	# print("commanded vel: ", cur_vel +  acc*time)
	# print("cur del: ", cur_del)
	# print("commanded del: ", cur_vel +  delvel*time)
	print("----------")
	# Move the car autonomously
	command_pub.publish(msgtosend)
	
	

if __name__ == '__main__':
	
	# kp = input("Enter Kp Value: ")
	# kd = input("Enter Kd Value: ")
	# ki = input("Enter Ki Value: ")
	# vel_input = input("Enter desired velocity: ")
	rospy.init_node('pid_controller', anonymous=True)
	print("PID Control Node is Listening to error")
	delta_sub = message_filters.Subscriber('/mytf', tfheader, queue_size=1)
	pose_sub = message_filters.Subscriber('/mypose', poseheader, queue_size = 1)
	ts = message_filters.ApproximateTimeSynchronizer([delta_sub, pose_sub], 1, 0.01, allow_headerless=False)
	ts.registerCallback(control)
	rospy.spin()
