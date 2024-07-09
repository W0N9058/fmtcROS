#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pickle
import cv2
import os
from pynput import keyboard

class DataMaker:
    def __init__(self):
        self.bridge = CvBridge()
        self.image = None
        self.int_value = None
        
        # 카메라 정보 받아오기
        self.image_sub = rospy.Subscriber('/camera/image', Image, self.image_callback)
        # 조향값 정보 받아오기
        self.int_sub = rospy.Subscriber('/pot_angle', Int32, self.angle_callback)
        
        # 키보드 입력 받아주기
        self.keyboard_pub = rospy.Publisher('/keyboard_input', Int32, queue_size=10)
        
        # 저장 경로
        self.save_dir = rospy.get_param('~save_dir', '/tmp')
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        # 키보드 리스너 설정
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def angle_callback(self, msg):
        self.int_value = msg.data

    def generate_filename(self, base_path):
        index = 1
        file_path = f"{base_path}/data.pkl"
        while os.path.exists(file_path):
            file_path = f"{base_path}/data{index}.pkl"
            index += 1
        return file_path

    def save_data(self):
        if (self.image is not None) and (self.int_value is not None):
            data = {
                'image': self.image,
                'int_value': self.int_value
            }
            file_path = self.generate_filename(self.save_dir)
            with open(file_path, 'wb') as f:
                pickle.dump(data, f)
            rospy.loginfo(f"Data saved to {file_path}")
            
            # 자료가 전달되지 않았을때 기준 코드
            # 다만 이미지랑, 조향값이랑 초당 전송 개수가 달라서 문제가 발생할 수 있다. 해당 문제 발생시 아래 코드 수정
            self.image = None
            self.int_value = None

    def on_press(self, key):
        try:
            # 키보드 입력을 문자로 변환하여 퍼블리시
            if key.char == 'w':
                self.keyboard_pub.publish(1)
                rospy.loginfo("Published keyboard input: 1 (forward)")
            elif key.char == 'a':
                self.keyboard_pub.publish(2)
                rospy.loginfo("Published keyboard input: 2 (left)")
            elif key.char == 's':
                self.keyboard_pub.publish(3)
                rospy.loginfo("Published keyboard input: 3 (backward)")
            elif key.char == 'd':
                self.keyboard_pub.publish(4)
                rospy.loginfo("Published keyboard input: 4 (right)")
            elif key.char == 'p':
                rospy.loginfo("Saving data due to 'p' key press")
                self.save_data()
        except AttributeError:
            # 문자가 아닌 키 입력 무시
            pass

if __name__ == '__main__':
    rospy.init_node('DataMaker', anonymous=True)
    saver = DataMaker()
    rospy.spin()
