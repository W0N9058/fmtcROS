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
        
        # 저장 경로
        self.save_dir = '/home/fmtc/data'
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        # 데이터 저장을 위한 리스트
        self.data_list = []

        # 키보드 리스너 설정
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.store_data()

    def angle_callback(self, msg):
        self.int_value = msg.data
        self.store_data()

    def store_data(self):
        if self.image is not None and self.int_value is not None:
            self.data_list.append({'image': self.image, 'int_value': self.int_value})
            rospy.loginfo("Data appended to list")
            self.image = None
            self.int_value = None

    def generate_filename(self, base_path):
        index = 1
        file_path = f"{base_path}/data.pkl"
        while os.path.exists(file_path):
            file_path = f"{base_path}/data{index}.pkl"
            index += 1
        return file_path

    def save_data(self):
        if self.data_list:
            file_path = self.generate_filename(self.save_dir)
            with open(file_path, 'wb') as f:
                pickle.dump(self.data_list, f)
            rospy.loginfo(f"Data saved to {file_path}")
            # 저장 후 데이터 리스트 초기화
            self.data_list = []

    def load_data(self, file_path):
        try:
            with open(file_path, 'rb') as f:
                data = pickle.load(f)
            rospy.loginfo(f"Data loaded from {file_path}")
            for item in data:
                rospy.loginfo(f"Image shape: {item['image'].shape}, Int value: {item['int_value']}")
        except Exception as e:
            rospy.logerr(f"Failed to load data from {file_path}: {e}")

    def on_press(self, key):
        try:
            if key.char == 's':
                rospy.loginfo("Saving data due to 's' key press")
                self.save_data()
            elif key.char == 'l':
                file_path = '/mnt/data/data9.pkl'  # Adjust the path as necessary
                rospy.loginfo("Loading data due to 'l' key press")
                self.load_data(file_path)
        except AttributeError:
            # 문자가 아닌 키 입력 무시
            pass

if __name__ == '__main__':
    rospy.init_node('DataMaker', anonymous=True)
    saver = DataMaker()
    rospy.spin()

