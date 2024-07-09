#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, Polygon
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

def callback(data):
    # RViz에 좌표를 띄우기 위해 visualization_msgs/Marker 메시지 사용
    marker = Marker()
    marker.header.frame_id = "laser"  # RViz에서 보여줄 좌표의 좌표계 설정
    marker.type = Marker.POINTS
    marker.action = Marker.ADD
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.color = ColorRGBA(0.0, 0.0, 1.0, 1.0)  # 색상 (빨간색)
    marker.lifetime = rospy.Duration(1)  # RViz에서 띄워진 좌표가 유지될 시간 (초)

    for point in data.points:
        marker.points.append(point)

    # 마커를 발행하기 위해 발행자 생성
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    marker_pub.publish(marker)

def polygon_subscriber():
    # 노드 초기화 및 토픽 구독자 생성
    rospy.init_node('rviz_subscriber', anonymous=True)
    rospy.Subscriber('cluster_coordinate', Polygon, callback)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        polygon_subscriber()
    except rospy.ROSInterruptException:
        pass