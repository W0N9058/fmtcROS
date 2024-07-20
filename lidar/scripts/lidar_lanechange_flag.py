#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Polygon
from std_msgs.msg import Float64MultiArray
from lidar.msg import obstacle_detection 
import numpy as np

class ObstacleDetection:

    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("obstacle_detection")

        # Create a subscriber for the cluster coordinates and point numbers
        rospy.Subscriber('cluster_coordinate', Polygon, self.cluster_coord_callback)
        rospy.Subscriber('cluster_point_nums', Float64MultiArray, self.cluster_point_nums_callback)

        # Create a publisher for the obstacle flag
        self.obstacle_flag_pub = rospy.Publisher('obstacle_flag', obstacle_detection, queue_size=10)

        # Define variables for storing cluster coordinates and point numbers
        self.cluster_coordinates = []
        self.cluster_point_nums = []

        # Define variables for tracking obstacle detection
        self.count = 0
        self.obstacle_flag = 0
        self.y_coord = 0
        self.x_coord = 0

    def cluster_coord_callback(self, msg):
        # Process the cluster coordinates
        self.cluster_coordinates = [(point.x, point.y) for point in msg.points]
        self.check_obstacle()

    def cluster_point_nums_callback(self, msg):
        # Process the cluster point numbers
        self.cluster_point_nums = msg.data
        self.check_obstacle()

    def check_obstacle(self):
        # Check if the specified conditions for obstacle detection are met
        if len(self.cluster_coordinates) == len(self.cluster_point_nums):
            obstacle_detected = any((-1 <= x <= 1 and -1 < y <= 1 and num >= 10) for (x, y), num in zip(self.cluster_coordinates, self.cluster_point_nums))
        else:
            obstacle_detected = False

        if obstacle_detected:
            #self.count += 1
            # if self.count >= 10:
                # self.obstacle_flag = 1
                
                # Extract indices of elements that satisfy the obstacle_detected condition
            indices = [index for index, value in enumerate(self.cluster_coordinates) if (-1 <= value[0] <= 1 and -1 < value[1] <= 1 and self.cluster_point_nums[index] >= 10)]

                # If there are indices that satisfy the condition, find the index with the maximum y-coordinate
            if indices:
                max_y_index = max(indices, key=lambda index: self.cluster_coordinates[index][1])
                self.x_coord, self.y_coord = self.cluster_coordinates[max_y_index]
                # print(f"Obstacle detected at coordinates ({self.x_coord}, {self.y_coord})")
                self.print_distance_to_object()
                    
        else:
            # Reset the count and obstacle flag
            self.count = 0
            self.obstacle_flag = 0
            self.y_coord = 0
            self.x_coord = 0

    def print_distance_to_object(self):
        # Calculate the distance to the detected object
        distance = np.sqrt(self.x_coord**2 + self.y_coord**2)
        print(f"Distance to the detected object: {distance:.2f} meters")

    def publish_obstacle_flag(self):
        # Publish the obstacle flag
        obstacle_flag_msg = obstacle_detection()
        obstacle_flag_msg.flag = self.obstacle_flag
        obstacle_flag_msg.y_coord = self.y_coord
        self.obstacle_flag_pub.publish(obstacle_flag_msg)

def main():
    obstacle_detection = ObstacleDetection()
    rate = rospy.Rate(30)  # 10 Hz (adjust the rate as needed)
    while not rospy.is_shutdown():
        obstacle_detection.publish_obstacle_flag()
        rate.sleep()

if __name__ == "__main__":
    main()
