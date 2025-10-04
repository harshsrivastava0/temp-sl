#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Header
from slam.msg import stateHeader
from tf.transformations import euler_from_quaternion

pub = rospy.Publisher('/mystate', stateHeader, queue_size = 1)

def callback(msg):
        msgtosend = stateHeader()
        msgtosend.header = Header()
        msgtosend.header.frame_id = "velodyne"
        msgtosend.header.stamp = rospy.Time.now()
        msgtosend.pose = msg.pose[2].position
        msgtosend.yaw = (euler_from_quaternion([msg.pose[2].orientation.x, msg.pose[2].orientation.y, msg.pose[2].orientation.z, msg.pose[2].orientation.w])[2])
        # print(msgtosend.pose)
        # print(msgtosend.yaw)
        pub.publish(msgtosend)


if __name__ == '__main__':
    print("propogateState started")
    rospy.init_node('propogateState', anonymous = False)
    rospy.Subscriber("/gazebo/model_states", ModelStates, callback, queue_size=1)
    rospy.spin()
