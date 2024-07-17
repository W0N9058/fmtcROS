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

        # 데이터 저장을 위한 리스트
        self.data_img = []
        self.data_pot = []

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
                # 데이터 저장하고 수집 중지
                self.save_data()
                self.collecting = False
                rospy.loginfo("Collected data saved")
            else:
                # 데이터 수집 시작
                self.collecting = True
                rospy.loginfo("Started collecting data")

    def store_data(self):
        if self.image is not None and self.int_value is not None:
            self.data_img.append(self.image)
            self.data_pot.append(self.int_value)
            rospy.loginfo("Data appended to list")
            
            self.image = None
            self.int_value = None

    def generate_filename_img(self, base_path):
        index = 1
        file_path = f"{base_path}/data.mp4"
        while os.path.exists(file_path):
            file_path = f"{base_path}/data{index}.mp4"
            index += 1
        return file_path
        
    def generate_filename_pot(self, base_path):
        index = 1
        file_path = f"{base_path}/data.txt"
        while os.path.exists(file_path):
            file_path = f"{base_path}/data{index}.txt"
            index += 1
        return file_path

    def save_images_to_mp4(self, image_list, output_path, fps=30):
        """
        이미지 리스트를 받아 MP4 비디오로 저장합니다.

        :param image_list: 이미지를 포함하는 리스트
        :param output_path: 저장할 MP4 파일 경로
        :param fps: 비디오의 프레임 속도 (기본값 30)
        """
        if not image_list:
            raise ValueError("The image list is empty")
        
        # 이미지의 높이와 너비 가져오기
        height, width, _ = image_list[0].shape
        
        # 비디오 작성기 초기화
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))
        
        for image in image_list:
            out.write(image)
        
        out.release()
        rospy.loginfo(f"Video saved to {output_path}")
        
    def save_int_list_to_txt(self, int_list, output_path):
        """
        정수 리스트를 받아 텍스트 파일로 저장합니다.

        :param int_list: 정수를 포함하는 리스트
        :param output_path: 저장할 텍스트 파일 경로
        """
        with open(output_path, 'w') as file:
            for int_value in int_list:
                file.write(f"{int_value}\n")
        rospy.loginfo(f"Integer list saved to {output_path}")

    def save_data(self):
        if self.data_img and self.data_pot:
            file_path_img = self.generate_filename_img(self.save_dir)
            file_path_pot = self.generate_filename_pot(self.save_dir)
            
            self.save_images_to_mp4(self.data_img, file_path_img, fps=10)
            self.save_int_list_to_txt(self.data_pot, file_path_pot)
            
            rospy.loginfo(f"Data saved to {file_path_img} and {file_path_pot}")
            # 저장 후 데이터 리스트 초기화
            self.data_img = []
            self.data_pot = []

    def on_press(self, key):
        try:
            if key.char == 's':
                rospy.loginfo("Saving data due to 's' key press")
                self.save_data()
        except AttributeError:
            # 문자가 아닌 키 입력 무시
            pass

if __name__ == '__main__':
    rospy.init_node('DataMaker', anonymous=True)
    saver = DataMaker()
    rospy.spin()

