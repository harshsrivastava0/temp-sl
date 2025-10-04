#!/usr/bin/env python3
import rospy
import math
from ackermann_msgs.msg import AckermannDrive
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
import scipy
import numpy as np
import control as ct
from tf.transformations import euler_from_quaternion

current_states = []
x_pos = 0.0
y_pos = 0.0
prev_x = 0.0
prev_y = 0.0
cur_yaw = 0.0
delta = 0.0

vel = 0.0
theta = 0.0
beta = 0.0
wh_ba = 0.3302
lf = 0.1651
rf = 0.1651

prev_time = 0.0
prev_yaw = 0.0
time = 0.001

flagbbg = 0

def withinthresh(x, thresh):
    if(x>-thresh and x<thresh):
        return True
    else:
        return False

def callback1(data):
    global delta
    for i in data.transforms:
        if(i.child_frame_id=="front_left_wheel"):
            delta = (euler_from_quaternion([i.transform.rotation.x, i.transform.rotation.y, i.transform.rotation.z, i.transform.rotation.w])[2])
            

def callback2(data):
    global current_states, x_pos, y_pos, cur_yaw, delta, vel, theta, beta, wh_ba, lf, rf, prev_yaw, prev_time, time, flagbbg, prev_x, prev_y
    x_pos = data.pose.position.x
    y_pos = data.pose.position.y

    cur_yaw = euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])[2]
    if(cur_yaw<0):
        cur_yaw = 2*3.141592653589 + cur_yaw
    # current_states = np.array([[x_pos], [y_pos], [cur_yaw], [delta]])
    current_states = np.array([[x_pos], [y_pos], [cur_yaw]])
    # print(current_states)
    error()

def error():
    global current_states, x_pos, y_pos, cur_yaw, delta, vel, theta, beta, wh_ba, lf, rf, prev_yaw, prev_time, time, flagbbg, prev_x, prev_y
    # x_tf = np.array([[8.863594755591764], [-0.27720372143290856], [0.640488017891657], [0]])
    x_tf = np.array([[-3.2270596027374268], [16.58122444152832], [1.11966609954834]])
    error_states = -(x_tf-current_states)
    # print(error_states)
    # if(withinthresh(error_states[0][0], 0.2) and withinthresh(error_states[1][0], 0.2) and withinthresh(error_states[2][0], 5*3.14159265/180) and withinthresh(error_states[3][0], 5*3.14159265/180)):
    if(withinthresh(error_states[0][0], 0.2) and withinthresh(error_states[1][0], 0.2) and withinthresh(error_states[2][0], 5*3.14159265/180)):
        print("---------------------------------------------------------------------------------")
        control([[0],[0]])
        return

    if(prev_time==0.0):
        prev_time = rospy.get_time()
        return
    cur_time = rospy.get_time()
    time = cur_time-prev_time
    if(withinthresh(time, 0.05)):
        return
    prev_time = cur_time

    if(flagbbg):
        vel = (((current_states[0][0]-prev_x)/time)**2 + ((current_states[1][0]-prev_y)/time)**2)**0.5
    prev_x = current_states[0][0]
    prev_y = current_states[1][0]
    flagbbg = 1

    mybeta = math.atan2(math.tan(delta),2)


    R = np.array([[1,0],[0,1]])
    # Q = 100*np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    Q = np.array([[1,0,0],[0,1,0],[0,0,1]])
    


    #raman
    # A = np.array([[-1,0,-vel*time*math.sin(cur_yaw),0],[0,-1,vel*time*math.cos(cur_yaw),0],[0,0,-1,(vel/wh_ba)*((1/math.cos(delta))**2)*time],[0,0,0,-1]])
    # B = np.array( [ [math.cos(cur_yaw)*time,0],[time*math.sin(cur_yaw),0],[(math.tan(delta)*time)/wh_ba,0],[0,time]] )

    # simple chatgpt
    # A = np.array([[0,0,   0,          0],
    #               [0,0, vel,          0],
    #               [0,0,   0,(vel/wh_ba)],
    #               [0,0,   0,          0]])
    # B = np.array( [ [          1,          0],
    #                 [          0,          0],
    #                 [delta/wh_ba,          0],
    #                 [          0,          1]] )
    

    # #kalpak no slip
    # A = np.array([[0,0,-vel*math.sin(cur_yaw) - vel*(lf/wh_ba)*math.tan(delta)*math.cos(cur_yaw)],
    #               [0,0, vel*math.cos(cur_yaw) - vel*(lf/wh_ba)*math.tan(delta)*math.sin(cur_yaw)],
    #               [0,0, 0]])
    
    # B = np.array( [ [math.cos(cur_yaw) - (lf/wh_ba)*math.tan(delta)*math.sin(cur_yaw), -vel*(lf/wh_ba)*(1/math.cos(delta)**2)*math.sin(cur_yaw)],
    #                 [math.sin(cur_yaw) + (lf/wh_ba)*math.tan(delta)*math.cos(cur_yaw), vel*(lf/wh_ba)*(1/math.cos(delta)**2)*math.cos(cur_yaw)],
    #                 [math.tan(delta)/wh_ba, (vel/wh_ba)*(1/math.cos(delta)**2)]] )
    
    #gurkirat
    # A = np.array([[0,0,-vel*math.sin(cur_yaw) - vel*(lf/wh_ba)*math.tan(delta)*math.cos(cur_yaw)],
    #               [0,0, vel*math.cos(cur_yaw) - vel*(lf/wh_ba)*math.tan(delta)*math.sin(cur_yaw)],
    #               [0,0, 0]])
    
    # B = np.array( [ [math.cos(cur_yaw) - (lf/wh_ba)*math.tan(delta)*math.sin(cur_yaw), -vel*(lf/wh_ba)*(1/math.cos(delta)**2)*math.sin(cur_yaw)],
    #                 [math.sin(cur_yaw) + (lf/wh_ba)*math.tan(delta)*math.cos(cur_yaw), vel*(lf/wh_ba)*(1/math.cos(delta)**2)*math.cos(cur_yaw)],
    #                 [math.tan(delta)/wh_ba, (vel/wh_ba)*(1/math.cos(delta)**2)]] )

    #SDRE
    A = np.array([[0,0,   0],
                  [0,0,   0],
                  [0,0,   0]])
    B = np.array( [ [math.cos(mybeta+cur_yaw),                          0],
                    [math.sin(mybeta+cur_yaw),                          0],
                    [math.cos(mybeta)*math.tan(delta)/wh_ba,            0]] )
    
    print("curstates: ", "\n" ,current_states, end="\n\n")
    print("cur v,delta, beta: ", "\n" ,[[vel], [delta*180/3.141592653589], [mybeta*180/3.141592653589]], end="\n\n")
    print("B: ", "\n" ,B, end="\n\n")
    print("BRinvBt: ", "\n" , B@np.linalg.inv(R)@np.transpose(B), end="\n\n")
    print("Determinant of BRinvBt: ", "\n" , np.linalg.det(B@np.linalg.inv(R)@np.transpose(B)), end="\n\n")

    P = scipy.linalg.solve_continuous_are(A,B,Q,R)
    print("Solution to ARE: ", "\n" , P, end="\n\n")
    K = np.linalg.inv(R) @ np.transpose(B) @ P
    U = -1*np.dot(K,error_states)
    eigenvalues, eigenvectors = np.linalg.eig(A-(B@K))
    print("Eigen values of A-BK: ", "\n" , eigenvalues, end="\n\n");
    print("U Calculated: ", "\n" , U, end="\n\n");
    if(U[1][0]>24*3.141592653589/180):
        U[1][0]=24*3.141592653589/180
    if(U[1][0]<-24*3.141592653589/180):
        U[1][0]=-24*3.141592653589/180
    # print(U)
    # U[0][0]=U[0][0]*0.08
    # vel = U[0][0]
    # delta = U[1][0]
    control(U)

def control(input_array):
    # print(input_array)
    pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)
    command1 = AckermannDriveStamped()
    command = AckermannDrive()
    command.speed = input_array[0][0]
    command.steering_angle = input_array[1][0]
    command1.drive = command
    pub.publish(command1)


if __name__ == '__main__':
    print("bruh started")
    rospy.init_node('myLQR', anonymous = False)
    rospy.Subscriber("/tf", TFMessage, callback1)
    rospy.Subscriber("/gt_pose", PoseStamped, callback2)
    rospy.spin()
