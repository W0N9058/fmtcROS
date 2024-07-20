#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32, Bool, Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from pynput import keyboard

class DataMaker:
    def __init__(self):
        self.bridge = CvBridge()
        self.image = None
        self.steer_value = None
        self.collecting = False
        
        # Camera information subscriber
        self.image_sub = rospy.Subscriber('/camera/image', Image, self.image_callback)
        # Button input subscriber
        self.button_sub = rospy.Subscriber('/save_command', Bool, self.button_callback)
        # Controller input subscriber
        self.steering_sub = rospy.Subscriber('control_input', Float32MultiArray, self.steering_callback)
       
        # Save directory
        self.save_dir = '/home/fmtc/data'
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        # Data storage lists
        self.data_img = []
        self.data_pot = []

        # Keyboard listener setup
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if self.collecting:
            self.store_data()

    def button_callback(self, msg):
        if msg.data:
            if self.collecting:
                # Save data and stop collecting
                self.save_data()
                self.collecting = False
                rospy.loginfo("Collected data saved")
            else:
                # Start collecting data
                self.collecting = True
                rospy.loginfo("Started collecting data")
         
    def steering_callback(self, msg):
        if len(msg.data) >= 3:
            self.steer_value = msg.data[2]
        if self.collecting:
            self.store_data()

    def store_data(self):
        if self.image is not None and self.steer_value is not None:
            self.data_img.append(self.image)
            self.data_pot.append(self.steer_value)
            rospy.loginfo("Data appended to list")
            
            self.image = None
            self.steer_value = None

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
        Save the list of images as an MP4 video.

        :param image_list: List of images
        :param output_path: Path to save the MP4 file
        :param fps: Frames per second for the video (default is 30)
        """
        if not image_list:
            raise ValueError("The image list is empty")
        
        # Get the height and width of the images
        height, width, _ = image_list[0].shape
        
        # Initialize the video writer
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))
        
        for image in image_list:
            out.write(image)
        
        out.release()
        rospy.loginfo(f"Video saved to {output_path}")
        
    def save_int_list_to_txt(self, int_list, output_path):
        """
        Save the list of integers to a text file.

        :param int_list: List of integers
        :param output_path: Path to save the text file
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
            # Reset data lists after saving
            self.data_img = []
            self.data_pot = []

    def on_press(self, key):
        try:
            if key.char == 's':
                rospy.loginfo("Saving data due to 's' key press")
                self.save_data()
        except AttributeError:
            # Ignore non-character keys
            pass

if __name__ == '__main__':
    rospy.init_node('DataMaker', anonymous=True)
    saver = DataMaker()
    rospy.spin()

