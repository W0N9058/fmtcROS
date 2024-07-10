#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def image_publisher():
    # ROS 노드 초기화
    rospy.init_node('usb_cam_publisher', anonymous=True)
    # 이미지 퍼블리셔 설정
    image_pub = rospy.Publisher('/camera/image', Image, queue_size=10)
    bridge = CvBridge()

    # USB 카메라 열기 (디바이스 인덱스 0 사용)
    cap = cv2.VideoCapture(2)
    if not cap.isOpened():
        rospy.logerr("USB 카메라를 열 수 없습니다.")
        return

    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        # 프레임 읽기
        ret, frame = cap.read()
        if not ret:
            rospy.logerr("프레임을 읽을 수 없습니다.")
            continue

        # OpenCV 이미지를 ROS 이미지 메시지로 변환
        image_msg = bridge.cv2_to_imgmsg(frame, "bgr8")

        # 이미지 메시지 퍼블리시
        image_pub.publish(image_msg)
        rospy.loginfo("이미지 퍼블리시됨")

        # 루프 주기 유지
        rate.sleep()

    cap.release()

if __name__ == '__main__':
    try:
        image_publisher()
    except rospy.ROSInterruptException:
        pass
