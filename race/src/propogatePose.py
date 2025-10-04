#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from race.msg import poseheader
from std_msgs.msg import Header

pub = rospy.Publisher('/mypose', poseheader, queue_size = 1)

def callback(msg):
    msgtosend = poseheader()
    msgtosend.header = Header()
    msgtosend.header.frame_id = "myframe"
    msgtosend.header.stamp = rospy.Time.now()
    msgtosend.pose = msg.pose
    pub.publish(msgtosend)

if __name__ == '__main__':
    print("propogatePose started")
    rospy.init_node('propogatePose', anonymous = False)
    rospy.Subscriber("/gt_pose", PoseStamped, callback, queue_size=1)
    rospy.spin()