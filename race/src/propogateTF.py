#!/usr/bin/env python3
import rospy
from tf2_msgs.msg import TFMessage
from race.msg import tfheader
from std_msgs.msg import Header

pub = rospy.Publisher('/mytf', tfheader, queue_size = 1)

def callback(msg):
    for i in msg.transforms:
        if(i.child_frame_id=="front_left_wheel"):
            msgtosend = tfheader()
            msgtosend.header = Header()
            msgtosend.header.frame_id = "myframe"
            msgtosend.header.stamp = rospy.Time.now()
            msgtosend.transform = i.transform
            pub.publish(msgtosend)

if __name__ == '__main__':
    print("propogateTF started")
    rospy.init_node('propogateTF', anonymous = False)
    rospy.Subscriber("/tf", TFMessage, callback, queue_size=1)
    rospy.spin()