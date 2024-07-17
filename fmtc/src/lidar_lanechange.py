#!/usr/bin/env python3

#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Polygon
from std_msgs.msg import Float64MultiArray, Float64
from lidar.msg import obstacle_detection 
import numpy as np

class ObstacleDetection:

    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("obstacle_detection")

        # Create a subscriber for the cluster coordinates and point numbers
        rospy.Subscriber('cluster_coordinate', Polygon, self.cluster_coord_callback)
        rospy.Subscriber('cluster_point_nums', Float64MultiArray, self.cluster_point_nums_callback)
        
        # Create a publisher for the distance to the detected object
        self.distance_pub = rospy.Publisher('distance_to_object', Float64, queue_size=10)

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
            obstacle_detected = any((-1 <= x <= 1 and -0.6 < y <= 0.6 and num >= 20) for (x, y), num in zip(self.cluster_coordinates, self.cluster_point_nums))
        else:
            obstacle_detected = False

        if obstacle_detected:
        
            # Extract indices of elements that satisfy the obstacle_detected condition
            indices = [index for index, value in enumerate(self.cluster_coordinates) if (-1 <= value[0] <= 1 and -0.6 < value[1] <= 0.6 and self.cluster_point_nums[index] >= 20)]

            # If there are indices that satisfy the condition, find the index with the maximum y-coordinate
            if indices:
                max_y_index = max(indices, key=lambda index: self.cluster_coordinates[index][1])
                self.x_coord, self.y_coord = self.cluster_coordinates[max_y_index]
                self.print_distance_to_object()
                self.publish_distance()
                    
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

    def publish_distance(self):
        # Calculate the distance to the detected object
        distance = np.sqrt(self.x_coord**2 + self.y_coord**2)
        # Publish the distance to the detected object
        self.distance_pub.publish(distance)

def main():
    obstacle_detection = ObstacleDetection()
    rate = rospy.Rate(30)  # 30 Hz (adjust the rate as needed)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == "__main__":
    main()


