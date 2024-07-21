#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import cv2
import torch
import numpy as np
import time

def publish_traffic_light_status():
    # ROS node initialization
    rospy.init_node('traffic_light_detector', anonymous=True)
    
    # Publisher setup
    traffic_light_pub = rospy.Publisher('traffic_light', String, queue_size=10)
    
    # delay
    delay_time = 50  # 지연 시간(초)
    rospy.loginfo(f"Delaying YOLOv5 model load by {delay_time} seconds")
    time.sleep(delay_time)
    
    # Load YOLOv5 model
    model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/fmtc/yolov5/runs/train/exp3/weights/best.pt')  # change 'best.pt' to your trained model file

    # Camera capture
    cap = cv2.VideoCapture(20)

    if not cap.isOpened():
        rospy.logerr("Error: Cannot open video source")
        return

    rospy.loginfo("Starting traffic light detection")

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn("Failed to capture frame")
            continue

        # Inference with YOLOv5
        results = model(frame)

        # Get detected object information
        labels = results.xyxyn[0][:, -1].cpu().numpy()
        bboxes = results.xyxyn[0][:, :-1].cpu().numpy()
        names = results.names
        detected_classes = [names[int(label)] for label in labels]

        # Publish topic according to the traffic light status
        for i, detected_class in enumerate(detected_classes):
            if detected_class == 'traffic light':
                bbox = bboxes[i]
                x1, y1, x2, y2 = int(bbox[0] * frame.shape[1]), int(bbox[1] * frame.shape[0]), int(bbox[2] * frame.shape[1]), int(bbox[3] * frame.shape[0])
                traffic_light_pub.publish('traffic light')
                
                # Extract traffic light region
                traffic_light_region = frame[y1:y2, x1:x2]

                # Calculate bounding box size
                bbox_area = (x2 - x1) * (y2 - y1)
                specific_area_threshold = 13500  # Set the threshold for size condition (example)

                # Check if region size is sufficient
                height, width, _ = traffic_light_region.shape
                if width < 3 or height < 3:
                    rospy.logwarn("Traffic light region too small to process")
                    continue

                left_region = traffic_light_region[:, :width // 3]
                center_region = traffic_light_region[:, width // 3: 2 * width // 3]
                right_region = traffic_light_region[:, 2 * width // 3:]

                def get_center_points(region):
                    h, w, _ = region.shape
                    if h < 3 or w < 3:
                        return []
                    return [
                        region[h // 2 - 1, w // 2 - 1], region[h // 2, w // 2 - 1], region[h // 2 + 1, w // 2 - 1],
                        region[h // 2 - 1, w // 2], region[h // 2, w // 2], region[h // 2 + 1, w // 2],
                        region[h // 2 - 1, w // 2 + 1], region[h // 2, w // 2 + 1], region[h // 2 + 1, w // 2 + 1]
                    ]

                left_points = get_center_points(left_region)
                center_points = get_center_points(center_region)
                right_points = get_center_points(right_region)

                if not left_points or not center_points or not right_points:
                    rospy.logwarn("Region points insufficient for calculation")
                    continue

                left_avg = np.mean([np.mean(point) for point in left_points])
                center_avg = np.mean([np.mean(point) for point in center_points])
                right_avg = np.mean([np.mean(point) for point in right_points])

                # Publish 'stop' or 'go' if conditions are met
                if left_avg > center_avg and left_avg > right_avg and left_avg > 250 and bbox_area > specific_area_threshold:
                    traffic_light_pub.publish('stop')
                elif right_avg > center_avg and right_avg > left_avg and right_avg > 250 and bbox_area > specific_area_threshold:
                    traffic_light_pub.publish('go')

        # Show result frame (optional)
        #cv2.imshow('Traffic Light Detection', frame)
        #if cv2.waitKey(1) & 0xFF == ord('q'):
        #    break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        publish_traffic_light_status()
    except rospy.ROSInterruptException:
        pass

