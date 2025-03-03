#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def callback(msg):
    rospy.loginfo(f"Received: {msg.data}")
    # Process the message and send a response
    response_msg = String()
    response_msg.data = "Response from Raspberry Pi"
    pub.publish(response_msg)

if __name__ == '__main__':
    rospy.init_node('raspberry_pi_node')
    pub = rospy.Publisher('/response_topic', String, queue_size=10)
    sub = rospy.Subscriber('/testfc2', String, callback)
    rospy.loginfo("Bidirectional node is running...")
    rospy.spin()
