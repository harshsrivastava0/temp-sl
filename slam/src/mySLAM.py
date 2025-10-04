#!/usr/bin/env python3
import rospy
import math
from slam.msg import deltaHeader
from slam.msg import stateHeader
from clustering.msg import CoordinateList
from clustering.msg import Coordinates
import message_filters
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu

# measuredState: [x, y, delta, yaw, velocity, beta, yaw_rate]
# flag: [firsttime, match-flag-step1, prevPredicted]
# prevVar: [time, x, y, beta, yaw, velocity, yaw_rate]
# xT: [x, y, yaw]
# xTminus1: [x, y, yaw]
# uTminus1: [beta, velocity, yaw_rate]
# sigmaT: diagonal(x-x, y-y, yaw-yaw, x1-x1, y1-y1, x2-x2, y2-y2, ...) (NON DIAGONAL ELEMENTS ARE ALSO PRESENT)
# sigmaTminus1: diagonal(x-x, y-y, yaw-yaw, x1-x1, y1-y1, x2-x2, y2-y2, ...) (NON DIAGONAL ELEMENTS ARE ALSO PRESENT)

pubcone = rospy.Publisher('/coneVis', MarkerArray, queue_size=10)
pubpredictedpose = rospy.Publisher('/posePredictVis', Marker, queue_size=10)
pubactualpose = rospy.Publisher('/poseActualVis', Marker, queue_size=10)
measuredState = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
flag = [0, 0, 0]
prevVar = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
time = 0

receivedCoordinates = []
coneCounter = 0

xT = np.zeros((3+2*66, 1))
xTminus1 = np.zeros((3+2*66, 1))
intialpose = [0, 0, 0]
uTminus1 = np.zeros((3, 1))

Fx = np.zeros((3, 3+2*66))
Fx[0][0],  Fx[1][1], Fx[2][2]= 1, 1, 1

Q = np.zeros((2, 2))
Q[0][0], Q[1][1] = 0.008*0.008, 0.008*0.008
R = np.zeros((3+2*66, 3+2*66))
R[0][0], R[1][1], R[2][2] = 0.008*0.008, 0.008*0.008, 0.008*0.008


predictedX, predictedY, actualX, actualY = [], [], [], []


#----initialising covariance matrix -----
sigmaTminus1 = np.zeros((3+2*66, 3+2*66))
for i in range(3+2*66):
    for j in range(3+2*66):
        sigmaTminus1[i][j] = 1e-6
        if(i==j and i>2): sigmaTminus1[i][j] = 1e8
        # else : sigmaTminus1[i][j] = 1e-3
sigmaT = sigmaTminus1

def create_line_marker(xpoints, ypoints, color):
    if(len(xpoints)!=len(ypoints)): return
    marker = Marker()
    marker.header.frame_id = "velodyne"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "lines"
    marker.id = 1
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    
    
    marker.scale.x = 0.1  # Line width
    if(color==0): marker.color.r = 1.0
    else: marker.color.r = 0.0
    if(color==1): marker.color.g = 1.0
    else: marker.color.g = 0.0
    if(color==2): marker.color.b = 1.0
    else: marker.color.b = 0.0
    marker.color.a = 1.0  # Fully opaque
    
    marker.lifetime = rospy.Duration(0.5)  # Set lifetime to 10 seconds
    
    # Define points to form a line
    for i in range(len(xpoints)):
        p = Point()
        p.x = xpoints[i]
        p.y = ypoints[i]
        p.z = 0
        marker.points.append(p)
    
    return marker
def create_marker_array(xpoints, ypoints):
    if(len(xpoints)!=len(ypoints)): return
    marker_array = MarkerArray()
    marker = Marker()
    marker.header.frame_id = "velodyne"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "points"
    marker.id = 0
    marker.type = Marker.POINTS
    marker.action = Marker.ADD
    
    marker.scale.x = 0.3  # Point size
    marker.scale.y = 0.3
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0  # Fully opaque

    marker.lifetime = rospy.Duration(0.5)
    # Define some points
    for i in range(len(xpoints)):
        p = Point()
        p.x = xpoints[i]
        p.y = ypoints[i]
        p.z = 0
        marker.points.append(p)
    
    marker_array.markers.append(marker)
    return marker_array


def withinthresh(x, thresh):
    if(x>-thresh and x<thresh):return True
    return False

def callback(deltamsg, statemsg, imumsg, coordmsg):
    global measuredState, flag, prevVar, time, receivedCoordinates, coneCounter, xT, xTminus1, uTminus1, sigmaT, sigmaTminus1, Q, R, Fx, predictedX, predictedY, actualX, actualY, intialpose
    sum1, sum2 = np.zeros((2*66 + 3, 1)), np.zeros((2*66 + 3, 2*66 + 3))

    #------------time-manage-------------
    if(prevVar[0]==0.0):
        prevVar[0] = rospy.get_time()
        return
    time = rospy.get_time()-prevVar[0]
    if(withinthresh(time, 0.01)):
        return
    prevVar[0] = rospy.get_time()
    #------------------------------------


    #------------ coordinates --------------
    receivedCoordinates.clear()
    for i in coordmsg.ConeCoordinates: receivedCoordinates.append([i.x, i.y, (i.x**2 + i.y**2)**0.5, math.atan2(i.y, i.x)])


    #------------- x and y --------------
    prevVar[1] = measuredState[0]
    prevVar[2] = measuredState[1]
    measuredState[0] = statemsg.pose.x
    measuredState[1] = statemsg.pose.y

    #--------------- delta  --------------
    measuredState[2] = deltamsg.delta


    #--------------- beta ----------------
    prevVar[3] = measuredState[5]
    measuredState[5] = math.atan2(math.tan(measuredState[2]), 2) #assuming Lr = L/2


    #---------------- yaw -----------------
    # straight ahead is 0 (makes sense because x axis is ahead)
    prevVar[4] = measuredState[3]
    measuredState[3] = statemsg.yaw


    #-------------- yaw-rate --------------
    prevVar[6] = measuredState[6]
    measuredState[6] = imumsg.angular_velocity.z


    #------ ignoring the first time -------
    if(not flag[0]):
        flag[0] = 1
        return
    

    #-------------- velocity --------------
    prevVar[5] = measuredState[4]
    measuredState[4] = (((measuredState[0]-prevVar[1])/time)**2 + ((measuredState[1]-prevVar[2])/time)**2)**0.5



    #--------- handling variables ---------
    uTminus1[0][0] = prevVar[3]
    uTminus1[1][0] = prevVar[5]
    uTminus1[2][0] = prevVar[6]
    if(not flag[2]):
        # xTminus1[0][0] = prevVar[1]
        # xTminus1[1][0] = prevVar[2]
        # xTminus1[2][0] = prevVar[4]
        intialpose[0] = prevVar[1]
        intialpose[1] = prevVar[2]
        intialpose[2] = prevVar[4]
        flag[2] = 1



    #---------- prediction step -----------
    #-------------- states ---------------- 
    g = np.array([[uTminus1[1][0]*math.cos(xTminus1[2][0] + uTminus1[0][0])*time],
                  [uTminus1[1][0]*math.sin(xTminus1[2][0] + uTminus1[0][0])*time],
                  [uTminus1[2][0]*time]])
    xT = xTminus1 + (Fx.transpose()@g)
    if(xT[2][0]>=3.14159265): xT[2][0] = xT[2][0]-(3.14159265)
    if(xT[2][0]<=-3.14159265): xT[2][0] = xT[2][0]+(3.14159265)

    # ----------- covariances -------------
    G = np.identity(2*66 + 3)
    G[0][2] += -1*uTminus1[1][0]*math.sin(xTminus1[2][0] + uTminus1[0][0])*time
    G[1][2] += uTminus1[1][0]*math.cos(xTminus1[2][0] + uTminus1[0][0])*time
    sigmaT = G @ sigmaTminus1 @ G.transpose() + R

    # print("predicted sigmaT: ")
    # with np.printoptions(precision=2, suppress=True, formatter={'float': '{:0.2f}'.format}, linewidth=100):
    #     print(sigmaT[:10, :10])


    # #----------- correction step ----------
    # if(coneCounter==0):
    #     for i in range(0, 2*len(receivedCoordinates), 2):
    #         coneCounter+=1
    #         xT[i+3][0] = receivedCoordinates[i//2][2]*math.cos(xT[2][0] + receivedCoordinates[i//2][3]) + xT[0][0]
    #         xT[i+4][0] = receivedCoordinates[i//2][2]*math.sin(xT[2][0] + receivedCoordinates[i//2][3]) + xT[1][0]
    # else:

    #     for i in receivedCoordinates:

    #         zCap, q, delX, delY = np.zeros((2, 1)), 0, 0, 0
    #         H, Fxj = np.zeros((2, 5)), np.zeros((5, 2*66 + 3))
    #         Fxj[0][0], Fxj[1][1], Fxj[2][2] = 1, 1, 1

    #         #------ creating known data association -------
    #         rotatedx_of_cone = i[2]*math.cos(i[3]+xT[2][0])
    #         rotatedy_of_cone = i[2]*math.sin(i[3]+xT[2][0])
    #         globalEquivalent = [rotatedx_of_cone+xT[0][0], rotatedy_of_cone+xT[1][0], ((rotatedx_of_cone+xT[0][0])**2 + (rotatedy_of_cone+xT[1][0])**2)**0.5, math.atan2(rotatedy_of_cone+xT[1][0], rotatedx_of_cone+xT[0][0])]
            
    #         flag[1] = 0
            

    #         for j in range(0, 2*coneCounter, 2):

    #         #----------- cone already present -------------
    #             if(((globalEquivalent[0]-xT[j+3][0])**2 + (globalEquivalent[1]-xT[j+4][0])**2)**0.5 <= 1.5):
    #                 flag[1] = 1

    #                 delX = (xT[j+3][0]-xT[0][0])
    #                 delY = (xT[j+4][0]-xT[1][0])
    #                 q = (delX**2 + delY**2)
    #                 zCap[0][0], zCap[1][0] = q**0.5, math.atan2(delY, delX) - xT[2][0]
    #                 Fxj[3][3+j], Fxj[4][4+j] = 1, 1

    #                 break

    #         #--------------- new cone spotted --------------
    #         if(not flag[1]):
    #             print("not matched")
    #             print("local received: ", i[0], ", ", i[1])
    #             print("global equivalent: ", globalEquivalent[0], ", ", globalEquivalent[1])
    #             print("current cones: ")
    #             for j in range(0, 2*coneCounter, 2):
    #                 print(xT[j+3][0], ", ", xT[j+4][0])
    #             print("\n")

    #             xT[(2*coneCounter)+3][0] = globalEquivalent[0]
    #             xT[(2*coneCounter)+4][0] = globalEquivalent[1]
    #             Fxj[3][(2*coneCounter)+3], Fxj[4][(2*coneCounter)+4] = 1, 1
    #             coneCounter+=1

    #             delX = i[0]
    #             delY = i[1]
    #             q = i[2]**2
    #             zCap[0][0], zCap[1][0] = i[2], i[3]
                
                
    #         #-- computing 'H' matrix, and thus Kalman Gain --
    #         H[0][0], H[1][0] = -1*delX/(q**0.5), delY/q
    #         H[0][1], H[1][1] = -1*delY/(q**0.5), -1*delX/q
    #         H[0][2], H[1][2] = 0, -1
    #         H[0][3], H[1][3] = delX/(q**0.5), -1*delY/q
    #         H[0][4], H[1][4] = delY/(q**0.5), delX/q
    #         H = H @ Fxj
    #         K = sigmaT @ H.transpose() @ np.linalg.inv((H @ sigmaT @ H.transpose()) + Q)
    #         sum1 += K @ (np.array([[i[2]], [i[3]]]) - zCap)
    #         # print("predicted local: ")
    #         # print(zCap, "\n")
    #         # print("actual local: ")
    #         # print(np.array([[i[2]], [i[3]]]), "\n")
    #         # print("sum1: ")
    #         # for i in range(10): print(sum1[i][0])
    #         # print()
    #         sum2 += K @ H
    #         # print("\nH: ")
    #         # for i in range(10): print(H[0][i], H[1][i])
    #         # print("\nK: ")
    #         # for i in range(10): print(K[i][0], K[i][1])
    #         # print("\n\nsum2: ")
    #         # for i in range(10):
    #         #     for j in range(10):
    #         #         print(int(sum2[i][j]>=0.0001)*7, end=" ")
    #         #     print()


    # #------------- updating ----------------
    # # print("before updating:\n", xT)
    # xT = xT + sum1
    # # print(sum1[:3,:])
    # # sigmaT = (np.identity(2*66 + 3) - sum2) @ sigmaT

    # # print("\ncorrected sigmaT: ")
    # # with np.printoptions(precision=2, suppress=True, formatter={'float': '{:0.2f}'.format}, linewidth=100):
    # #     print(sigmaT[:10, :10])
    # print("\n", "-------------")
    # # print("after updating:\n", xT)

    # coneX, coneY = [], []
    # for i in range(0, 2*coneCounter, 2):
    #     coneX.append(xT[3+i][0])
    #     coneY.append(xT[4+i][0])
    # cones = create_marker_array(coneX, coneY)
    # pubcone.publish(cones)

    # predictedX.append(xT[0][0])
    # predictedY.append(xT[1][0])
    # actualX.append(measuredState[0]-intialpose[0])
    # actualY.append(measuredState[1]-intialpose[1])

    # predictedpose = create_line_marker(predictedX, predictedY, 0)
    # actualpose = create_line_marker(actualX, actualY, 2)
    # pubpredictedpose.publish(predictedpose)
    # pubactualpose.publish(actualpose)
    


    #-------------- loop closure --------------    
    xTminus1 = xT
    sigmaTminus1 = sigmaT
    

    #------------- print-states ------------
    # print("\nPrev State: ")
    # print("\nx: ", prevVar[1], "\ny: ",prevVar[2], "\nyaw: ", prevVar[4], "\ndelta: noclue", "\nvelocity: ", prevVar[5], "\nbeta: ", prevVar[3], "\nyaw rate: ", prevVar[6], "\n")
    print("\nActual: ")
    print("\nx: ", measuredState[0]-intialpose[0], "\ny: ",measuredState[1]-intialpose[1], "\nyaw: ", measuredState[3]-intialpose[2], "\n")
    #     #   "\ndelta: ", measuredState[2], "\nvelocity: ", measuredState[4], "\nbeta: ", measuredState[5], "\nyaw rate: ", measuredState[6], "\n")
    print("\nPredicted: ")
    print("\nx: ", xT[0][0], "\ny: ",xT[1][0], "\nyaw: ", xT[2][0])

    # print("\nerror: ")
    # print("\nx: ", measuredState[0]-intialpose[0]-xT[0][0], "\ny: ", measuredState[1]-intialpose[1] - xT[1][0], "\nyaw: ", measuredState[3]-intialpose[2]-xT[2][0], "\n")
    print("----------------")

if __name__ == '__main__':
    print("mySLAM started")
    rospy.init_node('mySLAM', anonymous = False)
    delta_sub = message_filters.Subscriber('/mydelta', deltaHeader, queue_size=1)
    pose_sub = message_filters.Subscriber('/mystate', stateHeader, queue_size = 1)
    imu_sub = message_filters.Subscriber('/imu', Imu, queue_size = 1)
    lidar_sub = message_filters.Subscriber('/Clusters', CoordinateList, queue_size = 1)
    ts = message_filters.ApproximateTimeSynchronizer([delta_sub, pose_sub, imu_sub, lidar_sub], 1, 0.01, allow_headerless=False)
    ts.registerCallback(callback)
    rospy.spin()
