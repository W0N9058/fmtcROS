#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32, Bool
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import pickle
import cv2
import os


class Resnet_Pilot:
    def __init__(self):
        self.bridge = CvBridge()
        self.image = None
        # 카메라 정보 받아오기
        self.image_sub = rospy.Subscriber('/camera/image', Image, self.image_callback)
        self.image2_sub = rospy.Subscriber('/camera/image2', Image, self.image_callback)
        
        # 조향값
        self.pub = rospy.Publisher("control_input", Float32MultiArray, queue_size=10)
        
        self.forward  = - 1.0
        self.backward = 1.0
        self.angle    = 0.0

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    
    def PublishControlInput(self):
        msg = Float32MultiArray()
        msg.data = [self.forward, self.backward, self.angle]
        self.pub.publish(msg)
    
    def ModelWork(self):
##################### 이부분 수정하시면 됩니다! ########################


####################################################################
        self.angle = 

if __name__ == '__main__':
    rospy.init_node('Resnet_Pilot', anonymous=True)
    rate = rospy.Rate(100)
    Pilot = Resnet_Pilot()
    rospy.spin()
    
    while not rospy.is_shutdown():
        Pilot.ModelWork()
        Pilot.PublishControlInput()
        rate.sleep()