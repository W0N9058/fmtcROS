#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Polygon
from std_msgs.msg import Float64MultiArray
import numpy as np
from sklearn.cluster import DBSCAN
import time

class LidarProcessing:

    def __init__(self):
        rospy.init_node("lidar_processing")
        self.coord_pub = rospy.Publisher('cluster_coordinate', Polygon, queue_size=10)
        self.num_pub = rospy.Publisher('cluster_point_nums', Float64MultiArray, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        
        self.lidar_msg = LaserScan()
        self.threshold_short = 600
        self.threshold_mid = 1500
        self.threshold_far = 3000
        self.eps_short, self.eps_mid, self.eps_far = 100, 150, 200
        self.n_short, self.n_mid, self.n_far = 50, 30, 20

    def publish_coordinates_and_num(self, coordinates_list, dist_msg):
        polygon_msg = Polygon()
        dist = Float64MultiArray()
        dist.data = dist_msg
        for cord in coordinates_list:
            point_msg = Point()
            point_msg.x, point_msg.y, point_msg.z = cord[0], cord[1], 0
            polygon_msg.points.append(point_msg)
        self.coord_pub.publish(polygon_msg)
        self.num_pub.publish(dist)

    def clustering(self, x, y, eps, n):
        dbscan = DBSCAN(eps=eps, min_samples=n)
        coordinates = np.column_stack((x, y))
        clusters = dbscan.fit_predict(coordinates)
        return clusters, coordinates

    def polar_to_cartesian(self, angles, nums):
        x = nums * np.cos(np.radians(angles))
        y = nums * np.sin(np.radians(angles))
        return x, y

    def separate_points(self, qualities, nums, angles):
        short_idx = (qualities >= 7) & (nums < self.threshold_short)
        mid_idx = (qualities >= 7) & (self.threshold_short <= nums) & (nums <= self.threshold_mid)
        far_idx = (qualities >= 7) & (self.threshold_mid < nums) & (nums <= self.threshold_far)

        return angles[short_idx], nums[short_idx], angles[mid_idx], nums[mid_idx], angles[far_idx], nums[far_idx]

    def calculate_cluster_nums(self, clusters, coordinates):
        cluster_centers = []
        cluster_nums = []

        unique_clusters = np.unique(clusters[clusters >= 0])
        for cluster in unique_clusters:
            cluster_points = coordinates[clusters == cluster]
            cluster_center = np.mean(cluster_points, axis=0)
            cluster_num = len(cluster_points)
            cluster_centers.append(cluster_center)
            cluster_nums.append(cluster_num)

        return cluster_centers, cluster_nums

    def range_clustering(self, x_short, y_short, x_mid, y_mid, x_far, y_far):
        clusters_short, clusters_mid, clusters_far = [], [], []
        if len(x_short) > 0:
            clusters_short, coords_short = self.clustering(x_short, y_short, eps=self.eps_short, n=self.n_short)
        if len(x_mid) > 0:
            clusters_mid, coords_mid = self.clustering(x_mid, y_mid, eps=self.eps_mid, n=self.n_mid)
        if len(x_far) > 0:
            clusters_far, coords_far = self.clustering(x_far, y_far, eps=self.eps_far, n=self.n_far)

        center_group = []
        num_group = []

        if len(clusters_short) > 0:
            centers, nums = self.calculate_cluster_nums(clusters_short, coords_short)
            center_group.extend(centers)
            num_group.extend(nums)
        if len(clusters_mid) > 0:
            centers, nums = self.calculate_cluster_nums(clusters_mid, coords_mid)
            center_group.extend(centers)
            num_group.extend(nums)
        if len(clusters_far) > 0:
            centers, nums = self.calculate_cluster_nums(clusters_far, coords_far)
            center_group.extend(centers)
            num_group.extend(nums)

        return center_group, num_group

    def lidar_callback(self, msg):
        self.lidar_msg = msg
        start_time = time.time()

        angles = np.array([(msg.angle_min + msg.angle_increment * index) * 180 / np.pi for index in range(len(msg.ranges))])
        nums = np.array([dist * 1000 for dist in msg.ranges])
        qualities = np.array(msg.intensities)

        filtered_angles_short, filtered_nums_short, filtered_angles_mid, filtered_nums_mid, filtered_angles_far, filtered_nums_far = self.separate_points(qualities, nums, angles)

        x_short, y_short = self.polar_to_cartesian(filtered_angles_short, filtered_nums_short)
        x_mid, y_mid = self.polar_to_cartesian(filtered_angles_mid, filtered_nums_mid)
        x_far, y_far = self.polar_to_cartesian(filtered_angles_far, filtered_nums_far)

        center_group, num_group = self.range_clustering(x_short, y_short, x_mid, y_mid, x_far, y_far)
        self.publish_coordinates_and_num(center_group, num_group)

        elapsed_time = time.time() - start_time
        rospy.loginfo(f"Time: {elapsed_time:.10f}")

def main():
    LidarProcessing()
    rospy.spin()

if __name__ == "__main__":
    main()
