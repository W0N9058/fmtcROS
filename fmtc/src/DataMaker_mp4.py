#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from pynput import keyboard

class DataMaker:
    def __init__(self):
        self.bridge = CvBridge()
        self.image = None
        self.int_value = None
        self.collecting = False
        self.video_writer = None
        self.angle_file = None

        # 카메라 정보 받아오기
        self.image_sub = rospy.Subscriber('/camera/image', Image, self.image_callback)
        # 조향값 정보 받아오기
        self.int_sub = rospy.Subscriber('/pot_angle', Int32, self.angle_callback)
        # 버튼 인풋 정보 받아오기
        self.button_sub = rospy.Subscriber('/save_command', Bool, self.button_callback)
       
        # 저장 경로
        self.save_dir = '/home/fmtc/data'
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        # 키보드 리스너 설정
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if self.collecting:
            self.store_data()

    def angle_callback(self, msg):
        self.int_value = msg.data
        if self.collecting:
            self.store_data()

    def button_callback(self, msg):
        if msg.data:
            if self.collecting:
                self.save_data()
                self.collecting = False
                rospy.loginfo("Collecting data saved")
            else:
                self.collecting = True
                rospy.loginfo("Started collecting data")
                self.start_new_recording()

    def start_new_recording(self):
        video_file_path = self.generate_filename(self.save_dir, "video.avi")
        self.angle_file_path = self.generate_filename(self.save_dir, "angles.txt")
        self.video_writer = cv2.VideoWriter(video_file_path, cv2.VideoWriter_fourcc(*'XVID'), 10, (640, 480))
        self.angle_file = open(self.angle_file_path, 'w')

    def store_data(self):
        if self.image is not None and self.int_value is not None:
            self.video_writer.write(self.image)
            self.angle_file.write(f"{self.int_value}\n")
            rospy.loginfo("Data appended to list")
            self.image = None
            self.int_value = None

    def generate_filename(self, base_path, file_name):
        index = 1
        file_path = os.path.join(base_path, file_name)
        while os.path.exists(file_path):
            file_path = os.path.join(base_path, f"{file_name.split('.')[0]}_{index}.{file_name.split('.')[1]}")
            index += 1
        return file_path

    def save_data(self):
        if self.video_writer:
            self.video_writer.release()
            self.video_writer = None
        if self.angle_file:
            self.angle_file.close()
            self.angle_file = None
        rospy.loginfo(f"Data saved to {self.angle_file_path}")

    def load_data(self, video_path, angles_path):
        try:
            cap = cv2.VideoCapture(video_path)
            with open(angles_path, 'r') as f:
                angles = f.readlines()

            index = 0
            while cap.isOpened():
                ret, frame = cap.read()
                if not ret:
                    break
                angle = angles[index].strip()
                rospy.loginfo(f"Frame {index}: Angle {angle}")
                index += 1

            cap.release()
            rospy.loginfo(f"Data loaded from {video_path} and {angles_path}")
        except Exception as e:
            rospy.logerr(f"Failed to load data: {e}")

    def on_press(self, key):
        try:
            if key.char == 's':
                rospy.loginfo("Saving data due to 's' key press")
                self.save_data()
            elif key.char == 'l':
                video_path = '/mnt/data/video_1.avi'  # Adjust the path as necessary
                angles_path = '/mnt/data/angles_1.txt'  # Adjust the path as necessary
                rospy.loginfo("Loading data due to 'l' key press")
                self.load_data(video_path, angles_path)
        except AttributeError:
            # 문자가 아닌 키 입력 무시
            pass

if __name__ == '__main__':
    rospy.init_node('DataMaker', anonymous=True)
    saver = DataMaker()
    rospy.spin()

