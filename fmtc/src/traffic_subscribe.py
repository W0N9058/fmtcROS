#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("Detected: %s", data.data)

def listener():
    # ROS 노드 초기화
    rospy.init_node('traffic_light_listener', anonymous=True)
    
    # 토픽 구독
    rospy.Subscriber('traffic_light', String, callback)
    
    # 계속 대기
    rospy.spin()

if __name__ == '__main__':
    listener()
