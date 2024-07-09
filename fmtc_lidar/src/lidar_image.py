#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, Polygon
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

def callback(data):
    marker = Marker()
    marker.header.frame_id = "laser"
    marker.type = Marker.POINTS
    marker.action = Marker.ADD
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.color = ColorRGBA(0.0, 0.0, 1.0, 1.0)
    marker.lifetime = rospy.Duration(1)

    for point in data.points:
        marker.points.append(point)

    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    marker_pub.publish(marker)

def polygon_subscriber():
    rospy.init_node('rviz_subscriber', anonymous=True)
    rospy.Subscriber('cluster_coordinate', Polygon, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        polygon_subscriber()
    except rospy.ROSInterruptException:
        pass
