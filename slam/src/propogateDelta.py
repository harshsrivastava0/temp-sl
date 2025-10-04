#!/usr/bin/env python3
import rospy
from tf2_msgs.msg import TFMessage
from slam.msg import deltaHeader
from std_msgs.msg import Header
from tf.transformations import euler_from_quaternion

pub = rospy.Publisher('/mydelta', deltaHeader, queue_size = 1)

def callback(msg):
    for i in msg.transforms:
        if(i.child_frame_id=="left_front_axle_carrier"):
            msgtosend = deltaHeader()
            msgtosend.header = Header()
            msgtosend.header.frame_id = "velodyne"
            msgtosend.header.stamp = rospy.Time.now()
            msgtosend.delta = (euler_from_quaternion([i.transform.rotation.x, i.transform.rotation.y, i.transform.rotation.z, i.transform.rotation.w])[2])
            pub.publish(msgtosend)

if __name__ == '__main__':
    print("propogateDelta started")
    rospy.init_node('propogateDelta', anonymous = False)
    rospy.Subscriber("/tf", TFMessage, callback, queue_size=1)
    rospy.spin()