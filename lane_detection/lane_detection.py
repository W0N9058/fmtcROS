import cv2
import numpy as np
import os
from matplotlib import pyplot as plt

# Defining variables to hold meter-to-pixel conversion
ym_per_pix = 1.78 / 356
xm_per_pix = 2.03 / 406

PATH = os.getcwd()

def readVideo():
    inputVideo = cv2.VideoCapture(os.path.join(PATH, '0714dark.mp4'))
    
    length = int(inputVideo.get(cv2.CAP_PROP_FRAME_COUNT))
    width = int(inputVideo.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(inputVideo.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = inputVideo.get(cv2.CAP_PROP_FPS)

    print("length :", length)
    print("width :", width)
    print("height :", height)
    print("fps :", fps)
    
    return length, width, height, fps, inputVideo

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

def processImage(inputImage):

    # Apply HLS color filtering to filter out white lane lines
    hls = cv2.cvtColor(inputImage, cv2.COLOR_BGR2HLS)
    lower_white = np.array([0, 160, 10])
    upper_white = np.array([230, 230, 230])
    mask = cv2.inRange(inputImage, lower_white, upper_white)
    hls_result = cv2.bitwise_and(inputImage, inputImage, mask = mask)

    # Convert image to grayscale, apply threshold, blur & extract edges
    gray = cv2.cvtColor(hls_result, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 160, 255, cv2.THRESH_BINARY)
    blur = cv2.GaussianBlur(thresh,(5, 5), 0)
    canny = cv2.Canny(blur, 40, 60)

    # Display the processed images
    cv2.imshow("Image", inputImage)
    cv2.imshow("HLS Filtered", hls_result)
    cv2.imshow("Grayscale", gray)
    cv2.imshow("Thresholded", thresh)
    cv2.imshow("Blurred", blur)
    cv2.imshow("Canny Edges", canny)

    return hls_result, gray, thresh, blur, canny

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
    dst = np.float32([[142, 54],
                      [498, 54],
                      [142, 460],
                      [498, 460]])

    # Matrix to warp the image for birdseye window
    matrix = cv2.getPerspectiveTransform(src, dst)
    # Inverse matrix to unwarp the image for final window
    minv = cv2.getPerspectiveTransform(dst, src)
    birdseye = cv2.warpPerspective(inputImage, matrix, img_size)

    # Get the birdseye window dimensions
    height, width = birdseye.shape[:2]

    # Divide the birdseye view into 2 halves to separate left & right lanes
    birdseyeLeft  = birdseye[0:height, 0:width // 2]
    birdseyeRight = birdseye[0:height, width // 2:width]

    # Display birdseye view image
    # cv2.imshow("Birdseye" , birdseye)
    # cv2.imshow("Birdseye Left" , birdseyeLeft)
    # cv2.imshow("Birdseye Right", birdseyeRight)

    return birdseye, birdseyeLeft, birdseyeRight, minv

def plotHistogram(inputImage):

    histogram = np.sum(inputImage[inputImage.shape[0] // 2:, :], axis = 0)/255

    midpoint = np.floor(histogram.shape[0] / 2).astype(int)
    leftxBase = np.argmax(histogram[:midpoint])
    rightxBase = np.argmax(histogram[midpoint:]) + midpoint

    # plt.xlabel("Image X Coordinates")
    # plt.ylabel("Number of White Pixels")

    # Return histogram and x-coordinates of left & right lanes to calculate
    # lane width in pixels
    return histogram, leftxBase, rightxBase

def slide_window_search(binary_warped, histogram):

    # Find the start of left and right lane lines using histogram info
    out_img = np.dstack((binary_warped, binary_warped, binary_warped)) * 255
    midpoint = int(histogram.shape[0] / 2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    # A total of 9 windows will be used
    nwindows = 9
    window_height = int(binary_warped.shape[0] / nwindows)
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    leftx_current = leftx_base
    rightx_current = rightx_base
    margin = 75
    minpix = 30
    left_lane_inds = []
    right_lane_inds = []

    #### START - Loop to iterate through windows and search for lane lines #####
    for window in range(nwindows):
        win_y_low = binary_warped.shape[0] - (window + 1) * window_height
        win_y_high = binary_warped.shape[0] - window * window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin
        cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high),
        (0,255,0), 2)
        cv2.rectangle(out_img, (win_xright_low,win_y_low), (win_xright_high,win_y_high),
        (0,255,0), 2)
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
        (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
        (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0]
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)
        if len(good_left_inds) > minpix:
            leftx_current = int(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > minpix:
            rightx_current = int(np.mean(nonzerox[good_right_inds]))
    #### END - Loop to iterate through windows and search for lane lines #######

    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)

    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]

    # Apply 2nd degree polynomial fit to fit curves
    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)


    ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0])
    left_fitx = left_fit[0] * ploty**2 + left_fit[1] * ploty + left_fit[2]
    right_fitx = right_fit[0] * ploty**2 + right_fit[1] * ploty + right_fit[2]

    ltx = np.trunc(left_fitx)
    rtx = np.trunc(right_fitx)
    # plt.plot(right_fitx)
    # plt.show()

    out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
    out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]

    # plt.imshow(out_img)
    # plt.plot(left_fitx,  ploty, color = 'yellow')
    # plt.plot(right_fitx, ploty, color = 'yellow')
    # plt.xlim(0, 640)
    # plt.ylim(480, 0)

    '''
    fit 2nd polynomial x = f(y)
    ploty: 0 - 479 int
    left_fit: 3 coefficients for 2nd polynomial
    right_fit: 3 coefficients for 2nd polynomial
    ltx: int(f_left(ploty))
    rtx: int(f_right(ploty))
    '''
    return ploty, left_fit, right_fit, ltx, rtx

def general_search(binary_warped, left_fit, right_fit):

    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    margin = 75
    left_lane_inds = ((nonzerox > (left_fit[0]*(nonzeroy**2) + left_fit[1]*nonzeroy +
    left_fit[2] - margin)) & (nonzerox < (left_fit[0]*(nonzeroy**2) +
    left_fit[1]*nonzeroy + left_fit[2] + margin)))

    right_lane_inds = ((nonzerox > (right_fit[0]*(nonzeroy**2) + right_fit[1]*nonzeroy +
    right_fit[2] - margin)) & (nonzerox < (right_fit[0]*(nonzeroy**2) +
    right_fit[1]*nonzeroy + right_fit[2] + margin)))

    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]
    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)
    ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0])
    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]


    ## VISUALIZATION ###########################################################

    out_img = np.dstack((binary_warped, binary_warped, binary_warped))*255
    window_img = np.zeros_like(out_img)
    out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
    out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]

    left_line_window1 = np.array([np.transpose(np.vstack([left_fitx-margin, ploty]))])
    left_line_window2 = np.array([np.flipud(np.transpose(np.vstack([left_fitx+margin,
                                  ploty])))])
    left_line_pts = np.hstack((left_line_window1, left_line_window2))
    right_line_window1 = np.array([np.transpose(np.vstack([right_fitx-margin, ploty]))])
    right_line_window2 = np.array([np.flipud(np.transpose(np.vstack([right_fitx+margin, ploty])))])
    right_line_pts = np.hstack((right_line_window1, right_line_window2))

    cv2.fillPoly(window_img, np.int_([left_line_pts]), (0, 255, 0))
    cv2.fillPoly(window_img, np.int_([right_line_pts]), (0, 255, 0))
    result = cv2.addWeighted(out_img, 1, window_img, 0.3, 0)

    # plt.imshow(result)
    # plt.plot(left_fitx,  ploty, color = 'yellow')
    # plt.plot(right_fitx, ploty, color = 'yellow')
    # plt.xlim(0, 640)
    # plt.ylim(480, 0)

    ret = {}
    ret['leftx'] = leftx
    ret['rightx'] = rightx
    ret['left_fitx'] = left_fitx
    ret['right_fitx'] = right_fitx
    ret['ploty'] = ploty

    return ret

def measure_lane_curvature(ploty, leftx, rightx):

    leftx = leftx[::-1]  # Reverse to match top-to-bottom in y
    rightx = rightx[::-1]  # Reverse to match top-to-bottom in y

    # Choose the maximum y-value, corresponding to the bottom of the image
    y_eval = np.max(ploty)

    # Fit new polynomials to x, y in world space
    left_fit_cr = np.polyfit(ploty*ym_per_pix, leftx*xm_per_pix, 2)
    right_fit_cr = np.polyfit(ploty*ym_per_pix, rightx*xm_per_pix, 2)

    # Calculate the new radii of curvature
    left_curverad  = ((1 + (2*left_fit_cr[0]*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
    right_curverad = ((1 + (2*right_fit_cr[0]*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])
    # Now our radius of curvature is in meters
    # print(left_curverad, 'm', right_curverad, 'm')

    # Decide if it is a left or a right curve
    if leftx[0] - leftx[-1] > 60:
        curve_direction = 'Left Curve'
    elif leftx[-1] - leftx[0] > 60:
        curve_direction = 'Right Curve'
    else:
        curve_direction = 'Straight'

    return (left_curverad + right_curverad) / 2.0, curve_direction

def draw_lane_lines(original_image, warped_image, Minv, draw_info):

    leftx = draw_info['leftx']
    rightx = draw_info['rightx']
    left_fitx = draw_info['left_fitx']
    right_fitx = draw_info['right_fitx']
    ploty = draw_info['ploty']

    warp_zero = np.zeros_like(warped_image).astype(np.uint8)
    color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

    pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
    pts = np.hstack((pts_left, pts_right))

    mean_x = np.mean((left_fitx, right_fitx), axis=0)
    pts_mean = np.array([np.flipud(np.transpose(np.vstack([mean_x, ploty])))])

    cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))
    cv2.fillPoly(color_warp, np.int_([pts_mean]), (0, 255, 255))

    newwarp = cv2.warpPerspective(color_warp, Minv, (original_image.shape[1], original_image.shape[0]))
    result = cv2.addWeighted(original_image, 1, newwarp, 0.3, 0)

    return pts_mean, result

def offCenter(meanPts, inpFrame):

    # Calculating deviation in meters
    mpts = meanPts[-1][-1][-2].astype(int)
    pixelDeviation = inpFrame.shape[1] / 2 - abs(mpts)
    deviation = pixelDeviation * xm_per_pix
    direction = "left" if deviation < 0 else "right"

    return deviation, direction

def addText(img, radius, direction, deviation, devDirection):

    # Add the radius and center position to the image
    font = cv2.FONT_HERSHEY_TRIPLEX

    if (direction != 'Straight'):
        text = 'Radius of Curvature: ' + '{:04.0f}'.format(radius) + 'm'
        text1 = 'Curve Direction: ' + (direction)

    else:
        text = 'Radius of Curvature: ' + 'N/A'
        text1 = 'Curve Direction: ' + (direction)

    cv2.putText(img, text , (50,100), font, 0.8, (0,100, 200), 2, cv2.LINE_AA)
    cv2.putText(img, text1, (50,150), font, 0.8, (0,100, 200), 2, cv2.LINE_AA)

    # Deviation
    deviation_text = 'Off Center: ' + str(round(abs(deviation), 3)) + 'm' + ' to the ' + devDirection
    cv2.putText(img, deviation_text, (50, 200), cv2.FONT_HERSHEY_TRIPLEX, 0.8, (0,100, 200), 2, cv2.LINE_AA)

    return img

def main():
    length, width, height, fps, video = readVideo()
    count = 0
    while (video.isOpened()):
        ret, frame = video.read()
        
        # if count == 0:
        #     perspectiveWarp(frame)
        #     cv2.waitKey(0)
        
        if not ret:
            print(f"{ret}{count}: ignoring frame")
        else:
            # Calibration
            # cv2.imshow('before', frame)
            frame = Calibrate(frame)
            # cv2.imshow('after', frame)
            
            # Warping the frame
            birdView, birdViewL, birdViewR, minverse = perspectiveWarp(frame)
            cv2.imshow('birdView', birdView)
            
            # Lane detection
            hls, grayscale, thresh, blur, canny = processImage(birdView)
            hlsL, grayscaleL, threshL, blurL, cannyL = processImage(birdViewL)
            hlsR, grayscaleR, threshR, blurR, cannyR = processImage(birdViewR)
            
            cv2.imshow('canny', canny)
            
            # Plot histogram
            hist, leftBase, rightBase = plotHistogram(thresh)
            print(f"left: {leftBase} || mid: {leftBase + (rightBase - leftBase)/2} || right: {rightBase}")
            # plt.plot(hist)
            # plt.show()
            # cv2.waitKey(0)
            
            # Sliding windows search
            ploty, left_fit, right_fit, left_fitx, right_fitx = slide_window_search(thresh, hist)
            # plt.plot(right_fit)
            # plt.show()
            # cv2.waitKey(0)
            print(left_fit)
            cv2.waitKey(0)
            
            # General search
            draw_info = general_search(thresh, left_fit, right_fit)
            # plt.show()
            # cv2.waitKey(0)
            
            # Lane curvature
            curveRad, curveDir = measure_lane_curvature(ploty, left_fitx, right_fitx)
            
            # Filling the area of detected lanes with green
            meanPts, result = draw_lane_lines(frame, thresh, minverse, draw_info)

            # Off Center
            deviation, directionDev = offCenter(meanPts, frame)

            # Adding text to our final image
            finalImg = addText(result, curveRad, curveDir, deviation, directionDev)
            
            # Displaying final image
            cv2.imshow("Final", finalImg)
            
            
        if cv2.waitKey(50) == ord('q') or count >= length-1:
            break
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        
        count += 1

if __name__ == "__main__":
    main()