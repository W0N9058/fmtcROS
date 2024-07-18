import cv2
import numpy as np

def nothing():
    pass

def Calibrate(inputImage):
    # Parameters for calibration
    fx = 463.153
    fy = 457.514
    cx = 286.336
    cy = 232.420
    k1 = -0.186771
    k2 = 0.016192
    p1 = -0.010891
    p2 = 0.006875
    
    dist = np.array([k1, k2, p1, p2])

    mtx = np.array([[fx, 0, cx],
                    [0, fy, cy],
                    [0, 0, 1]])

    src = np.float32([[220, 155], [355, 155], [85, 400], [500, 400]])
    dst = np.float32([[160, 100], [480, 100], [160, 460], [480, 460]])
    
    # Apply camera calibration
    h, w = inputImage.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    inputImage = cv2.undistort(inputImage, mtx, dist, None, newcameramtx)
    x, y, w, h = roi
    
    return inputImage[y:y+h, x:x+w]

def perspectiveWarp(inputImage):

    # Get image size
    img_size = (640, 480)

    # inputImageCopied = inputImage.copy()
    # cv2.circle(inputImageCopied, (188, 145), 5, (0, 0, 255), -1)
    # cv2.circle(inputImageCopied, (306, 139), 5, (0, 0, 255), -1)
    # cv2.circle(inputImageCopied, (60, 363), 5, (0, 0, 255), -1)
    # cv2.circle(inputImageCopied, (430, 342), 5, (0, 0, 255), -1)
    # cv2.imshow('test', inputImageCopied)
    
    # Perspective points to be warped
    src = np.float32([[188, 145],
                      [306, 139],
                      [60, 363],
                      [430, 342]])

    # Window to be shown
    dst = np.float32([[100, 194],
                      [540, 194],
                      [100, 460],
                      [540, 460]])

    # Matrix to warp the image for birdseye window
    matrix = cv2.getPerspectiveTransform(src, dst)
    # Inverse matrix to unwarp the image for final window
    minv = cv2.getPerspectiveTransform(dst, src)
    birdseye = cv2.warpPerspective(inputImage, matrix, img_size)

    # Display birdseye view image
    # cv2.imshow("Birdseye" , birdseye)

    return birdseye, minv

cap = cv2.VideoCapture("data5.mp4")

cv2.namedWindow("Debugging")

cv2.createTrackbar("L-H", "Debugging", 0, 255, nothing)
cv2.createTrackbar("L-S", "Debugging", 0, 255, nothing)
cv2.createTrackbar("L-V", "Debugging", 0, 255, nothing)
cv2.createTrackbar("U-H", "Debugging", 0, 255, nothing)
cv2.createTrackbar("U-S", "Debugging", 0, 255, nothing)
cv2.createTrackbar("U-V", "Debugging", 0, 255, nothing)

count = 0
while (cap.isOpened()):
    ret, frame = cap.read()
    frame = Calibrate(frame)
    
    birdseye, minverse = perspectiveWarp(frame)
    cv2.imshow('birdseye', birdseye)

    hsv = cv2.cvtColor(birdseye, cv2.COLOR_BGR2HSV)

    l_h = cv2.getTrackbarPos("L-H", "Debugging")
    l_s = cv2.getTrackbarPos("L-S", "Debugging")
    l_v = cv2.getTrackbarPos("L-V", "Debugging")
    u_h = cv2.getTrackbarPos("U-H", "Debugging")
    u_s = cv2.getTrackbarPos("U-S", "Debugging")
    u_v = cv2.getTrackbarPos("U-V", "Debugging")

    # lower = np.array([l_h, l_s, l_v])
    # upper = np.array([u_h, u_s, u_v])
    
    lower = np.array([90, 0, 215])
    upper = np.array([255, 40, 255])

    mask = cv2.inRange(hsv, lower, upper)

    result = cv2.bitwise_and(birdseye, birdseye, mask=mask)

    cv2.imshow("frame", frame)
    cv2.imshow("hsv", hsv)
    cv2.imshow("mask", mask)
    cv2.imshow("result", result)

    key = cv2.waitKey(0)
    if key == 'q':
        break