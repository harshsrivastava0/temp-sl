#!/usr/bin/env python3
import rospy
import math
from ackermann_msgs.msg import AckermannDrive
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
from race.msg import tfheader
from race.msg import poseheader
import message_filters
import scipy
import numpy as np
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

# stateArray: [x, y, delta, velocity, yaw, yaw_dot, beta, beta_dot]
# inputs: [steering velocity, longitudinal acceleration]
# flag: [pose, delta, velocity, yaw, beta]
# prevVar: [time, x, y, delta, longitudinal velocity, yaw, beta]

stateArray = np.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])
inputArray = [0, 0]
flag = [0, 0, 0, 0, 0]
prevVar = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
commandedParam = [0, 0]
time = 0
waypointcounter = 0

mu = 0.523
hcg = 0.074
lr = 0.17145
lf = 0.15875
csf = 4.718
csr = 5.4562
m = 3.47
iz = 0.04712
g = 9.81

absmaxsteer = 24 #in degrees
minsteervel = 0
maxsteervel = 3.2 #rad/s
absmaxvelocity = 7 #m/s
minacc = -8.26 #m/s2
maxacc = 7.51 #m/s2

aconst = ((mu*m)/(iz*(lr+lf)))

def visualize_markers_2d(points, flag, frame_id="map", marker_topic="/visualization_marker"):
    """
    Visualize a set of 2D points as markers in RViz.
    
    Args:
        points (list of list): A 2D list of points [[x1, y1], [x2, y2], ...].
        frame_id (str): Frame ID for the marker (default is "map").
        marker_topic (str): Topic to publish the marker (default is "/visualization_marker").
    """
    if not points or not all(len(p) == 2 for p in points):
        rospy.logerr("Invalid points format. Must be a list of [x, y] pairs.")
        return
    
    # Initialize ROS node and publisher
    marker_pub = rospy.Publisher(marker_topic, Marker, queue_size=10)
    
    # Create a Marker message
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "points"  # Namespace for the marker
    marker.id = flag  # Unique ID for this marker
    marker.type = Marker.POINTS  # Use POINTS marker type
    marker.action = Marker.ADD  # Add/modify the marker
    marker.lifetime = rospy.Time(0.2 + (flag*0.1))
    # Set marker properties
    marker.scale.x = 0.1 + (flag*0.2)# Point width
    marker.scale.y = 0.1 + (flag*0.2)# Point height
    marker.color.a = 1.0  # Alpha (transparency)

    if(flag):
        marker.color.r = 1.0  # Red color
        marker.color.g = 0.0  # Green color
        marker.color.b = 0.0  # Blue color
    else:
        marker.color.r = 0.0  # Red color
        marker.color.g = 0.0  # Green color
        marker.color.b = 1.0  # Blue color
    
    # Add points to the marker
    for x, y in points:
        point = Point()
        point.x = x
        point.y = y
        point.z = 0.0  # Set Z to 0 for 2D visualization
        marker.points.append(point)
    
    marker.header.stamp = rospy.Time.now()  # Update timestamp
    marker_pub.publish(marker)


def withinthresh(x, thresh):
    if(x>-thresh and x<thresh):return True
    return False

def fsteer(delt, vdeltdesired):
    if((delt <= -math.radians(absmaxsteer) and vdeltdesired<0) or (delt >= math.radians(absmaxsteer) and vdeltdesired>0)): return 0
    if(vdeltdesired < minsteervel): return minsteervel/vdeltdesired
    if(vdeltdesired > maxsteervel): return maxsteervel/vdeltdesired
    return 1

def facc(velo, adesired):
    if((velo <= -absmaxvelocity and adesired<0) or (velo >= absmaxvelocity and adesired>0)): return 0
    if(adesired < minacc): return minacc/adesired
    if(adesired > maxacc): return maxacc/adesired
    return 1


def callback(deltamsg, posemsg):
    global stateArray, inputArray, flag, prevVar, time

    #------------time-manage-------------
    if(prevVar[0]==0.0):
        prevVar[0] = rospy.get_time()
        return
    time = rospy.get_time()-prevVar[0]
    if(withinthresh(time, 0.3)):
        return
    prevVar[0] = rospy.get_time()
    #------------------------------------


    #------------- x and y --------------
    stateArray[0][0] = posemsg.pose.position.x
    stateArray[1][0] = posemsg.pose.position.y


    #--------------- delta  --------------
    stateArray[2][0] = (euler_from_quaternion([deltamsg.transform.rotation.x, deltamsg.transform.rotation.y, deltamsg.transform.rotation.z, deltamsg.transform.rotation.w])[2])
    
    
    #------------ delta-velocity ---------
    if(flag[1]): inputArray[0] = (stateArray[2][0]-prevVar[3])/time
    flag[1] = 1
    prevVar[3] = stateArray[2][0]


    #-------------- velocity --------------
    if(flag[0]):stateArray[3][0] = (((stateArray[0][0]-prevVar[1])/time)**2 + ((stateArray[1][0]-prevVar[2])/time)**2)**0.5
    flag[0] = 1
    prevVar[1] = stateArray[0][0]
    prevVar[2] = stateArray[1][0]


    #---------------- yaw -----------------
    stateArray[4][0] = euler_from_quaternion([posemsg.pose.orientation.x, posemsg.pose.orientation.y, posemsg.pose.orientation.z, posemsg.pose.orientation.w])[2]
    if(stateArray[4][0]<0):stateArray[4][0] = 2*3.141592653589 + stateArray[4][0]


    #--------------- yaw-dot --------------
    if(flag[3]):stateArray[5][0] = (stateArray[4][0]-prevVar[5])/time
    flag[3] = 1
    prevVar[5] = stateArray[4][0]


    #----------------- beta ---------------
    stateArray[6][0] = math.atan2(math.tan(stateArray[2][0])*lr, lr+lf)


    #--------------- beta-dot -------------
    if(flag[4]): stateArray[7][0] = (stateArray[6][0] - prevVar[6])/time
    flag[4] = 1
    prevVar[6] = stateArray[6][0]


    #-------------- acceleration ----------
    if(flag[2]): inputArray[1] = (stateArray[3][0]*math.cos(stateArray[6][0]) - prevVar[4])/time
    flag[2] = 1
    prevVar[4] = stateArray[3][0]*math.cos(stateArray[6][0])


    #--------print-state and inputs--------
    print("delta: ", stateArray[2][0])
    # print("current STATE: ", "\nx: ", stateArray[0][0], "\ny: ", stateArray[1][0], "\ndelta: ", (stateArray[2][0]), "\nvelocity: ", stateArray[3][0], "\nyaw: ", (stateArray[4][0]))
    # print("yaw-dot: ", stateArray[5][0], "\nbeta: ", (stateArray[6][0]), "\nbeta_dot: ", stateArray[7][0], "\n")
    # print("current INPUTS: ", "\nvelocity-delta: ", inputArray[0], "\nlongitudinal-acceleration: ", inputArray[1], "\n")

    
    #------------ compute U ---------------
    compute()


def compute():
    global stateArray, inputArray, flag, prevVar, commandedParam, time, waypointcounter

    waypoints = [[0.0, 0.0],
                [0.03762573650077539, 0.38323937228042987],
                [0.07521233610311726, 0.7664771266969533],
                [0.11276295749370938, 1.1497132632495703],
                [0.1502809134415154, 1.5329475508148502],
                [0.18776936263320818, 1.916180066433944],
                [0.2252314637554933, 2.2994105789834314],
                [0.2626706066184526, 2.6826390884633233],
                [0.3000898728676407, 3.065865440791318],
                [0.33749257527202137, 3.449089558926309],
                [0.3748819495594074, 3.8323114428682628],
                [0.4122610773753644, 4.215530938534922],
                [0.44963334852997416, 4.5987479688851565],
                [0.48700192170993134, 4.981962379836665],
                [0.5243700326430597, 5.365174171389459],
                [0.561740917057172, 5.7483831894612685],
                [0.5991177336389629, 6.131589434052083],
                [0.6365037951573962, 6.514792751079623],
                [0.6739023373402846, 6.89799298646162],
                [0.7113165188743227, 7.281190217239213],
                [0.7487494984461944, 7.664384212288981],
                [0.7862046658659924, 8.047574894569786],
                [0.8236851798204112, 8.430762264081638],
                [0.8611941989961347, 8.813946166742237],
                [0.8987351132032557, 9.197126525510473],
                [0.9363109270461779, 9.580303263345174],
                [0.9739250303350269, 9.963476226164094],
                [1.0115805817564645, 10.346645491008351],
                [1.0492808170383143, 10.729810749713407],
                [1.0870289719084112, 11.112972079320379],
                [1.1248282820945787, 11.496129325747008],
                [1.1626819833246298, 11.879282411952158],
                [1.2005932342852481, 12.262431183853534],
                [1.2385652707042685, 12.645575718492289],
                [1.2766014053506434, 13.028715784745003],
                [1.3147047969110675, 13.411851305570536],
                [1.3528786811133648, 13.794982203927749],
                [1.3911259084796481, 14.178108479816627],
                [1.4294442448169131, 14.56123028731949],
                [1.4678270645872327, 14.944348165724266],
                [1.5062678963349483, 15.32746257727781],
                [1.5447600374810146, 15.71057406126808],
                [1.5832969395286332, 16.093683156983055],
                [1.6218720539810276, 16.476790326669562],
                [1.6604786782591305, 16.859896109615573],
                [1.6991103409072836, 17.243000968067914],
                [1.7377602623053123, 17.62610536427344],
                [1.7764220480386874, 18.009209837520082],
                [1.8150889955283525, 18.392314927095853],
                [1.8537545562775202, 18.775421172288695],
                [1.8924121047482738, 19.15852895830431],
                [1.9310550154026755, 19.541638824430663],
                [1.9696766627028195, 19.924751309955752],
                [2.0082705751930363, 20.3078669541675],
                [2.046830127335421, 20.690986142271637],
                [2.0853486165509056, 21.074109490597262],
                [2.1238194943427033, 21.457237384350073],
                [2.162236212214027, 21.840370439859157],
                [2.200592144626948, 22.223509119371386],
                [2.2388806660435616, 22.60665388513354],
                [2.277095227967059, 22.98980535347477],
                [2.3152292819006632, 23.37296390960077],
                [2.3532760482241777, 23.756130092799484],
                [2.3912291325230846, 24.1393044423589],
                [2.4290817551771977, 24.52248734348469],
                [2.4668275217719997, 24.905679489547133],
                [2.5044596526872827, 25.28888118871078],
                [2.54197159942627, 25.67209313434585],
                [2.5793568905333144, 26.055315634616914],
                [2.616608746388209, 26.438549305853094],
                [2.6537206955353168, 26.821794687342333],
                [2.6906860353955597, 27.205052164290347],
                [2.727498294513312, 27.588322353026246],
                [2.764151078474045, 27.97160571579685],
                [2.800643308701831, 28.354902252602173],
                [2.836977681636515, 28.738211809359914],
                [2.873157201882511, 29.121534231987813],
                [2.909185028126503, 29.504869289362457],
                [2.9450640108906043, 29.888216673319288],
                [2.980797077738111, 30.271576306817153],
                [3.0163873873556963, 30.654947881691527],
                [3.051837867306613, 31.038331243860092],
                [3.087151445154158, 31.42172608515833],
                [3.122331125502724, 31.805132328545085],
                [3.157379989997855, 32.188549588814666],
                [3.1923008891616855, 32.57197786596708],
                [3.227096904639759, 32.955416774796625],
                [3.2617710410364804, 33.33886616122102],
                [3.2963262259151236, 33.72232579411686],
                [3.330765463880104, 34.105795519401866],
                [3.365091836576943, 34.489275028911486],
                [3.3993081174866573, 34.87276416856343],
                [3.43341754233706, 35.256262707234285],
                [3.467422961650286, 35.63977049084179],
                [3.501327302989599, 36.02328721122137],
                [3.5351337250416828, 36.406812714290766],
                [3.5688451553698015, 36.79034676892655],
                [3.6024645215372404, 37.1738891440053],
                [3.6359948281483923, 37.55743968544475],
                [3.6694391568488016, 37.94099808508035],
                [3.702800358160614, 38.324564265870954],
                [3.7360815137293617, 38.70813784261085],
                [3.76928562815946, 39.09171873825893]]

    visualize_markers_2d(waypoints, 0)
    visualize_markers_2d([waypoints[waypointcounter]], 1)

    #1
    currentState = np.array([[stateArray[0][0]], [stateArray[1][0]], [stateArray[2][0]], [stateArray[3][0]], [stateArray[4][0]], [stateArray[5][0]], [stateArray[6][0]]])
    currentInput = [inputArray[0], inputArray[1]]

    #2
    # currentState = np.array([[stateArray[0][0]],
    #                          [stateArray[1][0]],
    #                          [stateArray[4][0]]])
    # currentInput = [stateArray[3][0], stateArray[2][0]]

    #3
    # currentState = np.array([[stateArray[0][0]],
    #                          [stateArray[1][0]],
    #                          [stateArray[4][0]], 
    #                          [stateArray[2][0]]])
    # currentInput = [stateArray[3][0], stateArray[2][0]]

    #4
    # currentState = np.array([[stateArray[0][0]],
    #                          [stateArray[1][0]],
    #                          [stateArray[4][0]]])
    # currentInput = [stateArray[3][0], stateArray[2][0]]

    #5 works
    # currentState = np.array([[stateArray[0][0]],
    #                          [stateArray[1][0]],
    #                          [stateArray[4][0]]])
    # currentInput = [stateArray[3][0], stateArray[2][0]]

    #1
    if(abs(currentState[3][0])>0.01):
        bconst = mu/(currentState[3][0]*(lr+lf))
        a1 = aconst*lf*csf*g*lr
        a2 = -aconst*(1/currentState[3][0])*((lf**2)*csf*g*lr + (lr**2)*csr*g*lf)
        a3 = aconst*(lr*csr*g*lf - lf*csf*g*lr)
        a4 = bconst*csf*g*lr
        a5 = bconst*(csr*g*lf*lr - csf*g*lr*lf)*(1/currentState[3][0]) - 1
        a6 = -bconst*(csr*g*lf + csf*g*lr)
        b1 = aconst*(-lf*csf*hcg*currentState[2][0] + lr*csr*hcg*currentState[6][0] + lf*csf*hcg*currentState[6][0] + (lf**2)*(csf*hcg*currentState[5][0]/currentState[3][0]) - (lr**2)*(csr*hcg*currentState[5][0]/currentState[3][0]))
        b2 = bconst*(-csf*hcg*currentState[2][0] - csr*hcg*currentState[6][0] + csf*hcg*currentState[6][0] + csr*hcg*lr*(currentState[5][0]/currentState[3][0]) + csf*hcg*lf*(currentState[5][0]/currentState[3][0]))
        A = np.array([[0, 0, 0, math.cos(currentState[4][0] + currentState[6][0]), 0, 0, 0], 
                    [0, 0, 0, math.sin(currentState[4][0] + currentState[6][0]), 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 1, 0],
                    [0, 0, a1, 0, 0, a2, a3],
                    [0, 0, a4, 0, 0, a5, a6]])
        B = np.array([[0, 0],
                    [0, 0],
                    # [fsteer(currentState[2][0], currentInput[0]), 0],
                    [1, 0],
                    # [0, facc(currentState[3][0], currentInput[1])],
                    [0, 1],
                    [0, 0],
                    [0, b1], 
                    [0, b2]])
    
    else:
        A = np.array([[0, 0, 0, math.cos(currentState[4][0] + currentState[6][0]), 0, 0, 0],
                     [0, 0, 0, math.sin(currentState[4][0] + currentState[6][0]), 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0],
                     [0, 0, 0, math.tan(currentState[2][0])*math.cos(currentState[6][0])/(lf+lr), 0, 0, 0],
                     [0, 0, 0, -(1/(lr+lf))*math.sin(currentState[6][0])*math.tan(currentState[2][0])*stateArray[7][0], 0, 0, 0],
                     [0, 0, 0, 0, 0, 0, 0]])
        B = np.array([[0, 0],
                     [0, 0],
                     [fsteer(currentState[2][0], currentInput[0]), 0],
                     [0, facc(currentState[3][0], currentInput[1])],
                     [0, 0],
                     [(1/(lr+lf))*fsteer(currentState[2][0], currentInput[0])*currentState[3][0]*math.cos(currentState[6][0])/(math.cos(currentState[2][0]))**2, (1/(lr+lf))*facc(currentState[3][0], currentInput[1])*math.cos(currentState[6][0])*math.tan(currentState[2][0])],
                     [fsteer(currentState[2][0], currentInput[0])*(lr/(lr+lf))*(1/math.cos(currentState[2][0])**2)*(1/(1+((lr/(lr+lf))*math.tan(currentState[2][0]))**2)), 0]])
    
    Q = np.array([[5000, 0, 0, 0, 0, 0, 0],
                [0, 100, 0, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0, 0],
                [0, 0, 0, 1, 0, 0, 0],
                [0, 0, 0, 0, 10, 0, 0],
                [0, 0, 0, 0, 0, 10, 0],
                [0, 0, 0, 0, 0, 0, 1]])
    R = np.array([[0.4,0],
                  [0,300]])

    #2
    # A = np.array([[0, 0, 0],
    #               [0, 0, 0],
    #               [0, 0, 0]])
    # B = np.array([[math.cos(currentState[2][0]), 0], 
    #               [math.sin(currentState[2][0]), 0],
    #               [(1/(lr+lf))*math.tan(currentInput[1]), 0]])
    # Q = np.eye(3)
    # R = np.eye(2)

    #3
    # A = np.array([[0, 0, 0, 0],
    #               [0, 0, currentInput[0], 0],
    #               [0, 0, 0, (currentInput[0]/(lr+lf))],
    #               [0,0, 0, 0]])
    # B = np.array( [ [1, 0],
    #                 [0, 0],
    #                 [currentInput[1]/(lr+lf), 0],
    #                 [0, 1]] )
    # Q = np.eye(4)
    # R = np.eye(2)

    #4
    # A = np.array([[0,0,-currentInput[0]*math.sin(currentState[2][0]) - currentInput[0]*(lf/(lr+lf))*math.tan(currentInput[1])*math.cos(currentState[2][0])],
    #               [0,0, currentInput[0]*math.cos(currentState[2][0]) - currentInput[0]*(lf/(lr+lf))*math.tan(currentInput[1])*math.sin(currentState[2][0])],
    #               [0,0, 0]])
    
    # B = np.array( [ [math.cos(currentState[2][0]) - (lf/(lr+lf))*math.tan(currentInput[1])*math.sin(currentState[2][0]), -currentInput[0]*(lf/(lr+lf))*(1/math.cos(currentInput[1])**2)*math.sin(currentState[2][0])],
    #                 [math.sin(currentState[2][0]) + (lf/(lr+lf))*math.tan(currentInput[1])*math.cos(currentState[2][0]), currentInput[0]*(lf/(lr+lf))*(1/math.cos(currentInput[1])**2)*math.cos(currentState[2][0])],
    #                 [math.tan(currentInput[1])/(lr+lf), (currentInput[0]/(lr+lf))*(1/math.cos(currentInput[1])**2)]] )
    # Q = np.eye(3)
    # R = np.eye(2)

    #5 works
    # A = np.array([[0,0,0],
    #               [0,0,currentInput[0]],
    #               [0,0, 0]])
    
    # B = np.array([[1, 0],
    #               [0, 0],
    #               [0, currentInput[0]/(lr+lf)]] )
    # Q = np.array([[3,0,0],
    #               [0,3,0],
    #               [0,0,1]])
    # R = np.array([[1,0],
    #               [0,2]])

    #1
    desiredState = np.array([[waypoints[waypointcounter][0]], [waypoints[waypointcounter][1]], [0.0], [0.0], [1.6], [0.0], [0.0]])

    #2
    # desiredState = np.array([[-6.666692733764648], [13.512182235717773], [0.5940194129943849]])

    #3
    # desiredState = np.array([[-6.666692733764648], [13.512182235717773], [0.5940194129943849], [0.0]])

    #4
    # desiredState = np.array([[-6.666692733764648], [13.512182235717773], [0.5940194129943849]])

    #5 works
    # desiredState = np.array([[-6.666692733764648], [13.512182235717773], [0]])

    errorState = -(desiredState-currentState)
    # print("X:\n", errorState, "\n")
    if(withinthresh(errorState[0][0], 6) and withinthresh(errorState[1][0], 0.5) and withinthresh(errorState[2][0], 30*3.14159265/180)):
        # print("---------------------------------------------------------------------------------")
        print("reached")
        waypointcounter+=5
        # control([[0],[0]])
        return
    
    
    # print("A:\n", A, "\n")
    # print("B:\n", B, "\n")
    # print("BRinvBt: ", "\n" , B@np.linalg.inv(R)@np.transpose(B), end="\n\n")
    # print("Determinant of BRinvBt: ", "\n" , np.linalg.det(B@np.linalg.inv(R)@np.transpose(B)), end="\n\n")

    try:
        P = scipy.linalg.solve_continuous_are(A,B,Q,R)
        K = np.linalg.inv(R) @ np.transpose(B) @ P
        U = -1*np.dot(K,errorState)
        # U[0][0] -= 0.25
        eigenvalues, eigenvectors = np.linalg.eig(A-(B@K))

        # print("Solution to ARE: ", "\n" , P, end="\n\n")
        # print("Eigen values of A-BK: ", "\n" , eigenvalues, end="\n\n");
        print("commanded INPUTS: ", "\nvelocity-delta: ", U[0][0], "\nlongitudinal-acceleration: ", U[1][0], "\n")
        print("chasing ", waypointcounter, "\n\n")
        # print("commanded INPUTS: ", "\nvelocity: ", U[0][0], "\ndelta: ", U[1][0], "\n")
        # print("-------------------------------")

        commandedParam[0] = currentState[2][0] + U[0][0]*time 
        commandedParam[1] = currentState[3][0] + U[1][0]*time
        # commandedParam[0] = U[0][0]
        # commandedParam[1] = U[1][0]
        control(commandedParam)
    except:
        print("------No finite solution-------")
        print("chasing ", waypointcounter, "\n\n")
        return



def control(input_array):
    pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)
    command1 = AckermannDriveStamped()
    command = AckermannDrive()
    command.steering_angle = input_array[0]
    command.speed = input_array[1]
    # command.steering_angle = input_array[1]
    # command.speed = input_array[0]
    command1.drive = command
    pub.publish(command1)


if __name__ == '__main__':
    print("newLQR started")
    rospy.init_node('newLQR', anonymous = False)
    delta_sub = message_filters.Subscriber('/mytf', tfheader, queue_size=1)
    pose_sub = message_filters.Subscriber('/mypose', poseheader, queue_size = 1)
    ts = message_filters.ApproximateTimeSynchronizer([delta_sub, pose_sub], 1, 0.01, allow_headerless=False)
    ts.registerCallback(callback)
    rospy.spin()
