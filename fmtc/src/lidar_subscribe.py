#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def object_detected_callback(msg):
    rospy.loginfo(f"Received message: {msg.data}")

def main():
    # Initialize the ROS node
    rospy.init_node('object_detected_listener', anonymous=True)

    # Create a subscriber for the 'object_detected' topic
    rospy.Subscriber('object_detected', String, object_detected_callback)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    main()
