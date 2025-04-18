#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray, String
from std_msgs.msg import Int32, Bool
from sensor_msgs.msg import Image
import pickle
import os
import threading
import torch
import torch.nn as nn
import torch.nn.functional as F
from torchvision import transforms
from PIL import Image as PILImage
import cv2
from cv_bridge import CvBridge


#################################### λͺ¨λΈ ? ?Έ ##############################

class SimpleResidualBlock(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv1 = nn.Conv2d(in_channels=3, out_channels=3, kernel_size=3, stride=1, padding=1)
        self.relu1 = nn.ReLU()
        self.conv2 = nn.Conv2d(in_channels=3, out_channels=3, kernel_size=3, stride=1, padding=1)
        self.relu2 = nn.ReLU()

    def forward(self, x):
        out = self.conv1(x)
        out = self.relu1(out)
        out = self.conv2(out)
        return self.relu2(out) + x # ReLU? ?? ₯? ?? ? ?? ?΄? ? ? ?©?  ? ??΅??€.


class ImageRegressionBase(nn.Module):
    def training_step(self, batch):
        images, labels = batch
        labels = labels.unsqueeze(1).float()  # ? ?΄λΈμ (batch_size, 1) ??λ‘? λ³???κ³? Float ?????Όλ‘? λ³?κ²?
        out = self(images)                  # ?μΈ? ??±
        loss = F.mse_loss(out, labels)      # ??€ κ³μ° (?κ·?λ₯? ?? ?κ·? ? κ³? ?€μ°?)
        return loss

    def validation_step(self, batch):
        images, labels = batch
        labels = labels.unsqueeze(1).float()  # ? ?΄λΈμ (batch_size, 1) ??λ‘? λ³???κ³? Float ?????Όλ‘? λ³?κ²?
        out = self(images)                   # ?μΈ? ??±
        loss = F.mse_loss(out, labels)       # ??€ κ³μ°
        return {"val_loss": loss.detach()}

    def validation_epoch_end(self, outputs):
        batch_losses = [x["val_loss"] for x in outputs]
        epoch_loss = torch.stack(batch_losses).mean()       # ??€ κ²°ν©
        return {"val_loss": epoch_loss} # ??€ κ²°ν©

    def epoch_end(self, epoch, result):
        print("Epoch [{}], last_lr: {:.5f}, train_loss: {:.4f}, val_loss: {:.4f}".format(
            epoch, result['lrs'][-1], result['train_loss'], result['val_loss']))


def ConvBlock(in_channels, out_channels, pool=False):
    layers = [nn.Conv2d(in_channels, out_channels, kernel_size=3, padding=1),
             nn.BatchNorm2d(out_channels),
             nn.ReLU(inplace=True)]
    if pool:
        layers.append(nn.MaxPool2d(2))
    return nn.Sequential(*layers)


class ResNet9(ImageRegressionBase):
    def __init__(self, in_channels):
        super().__init__()

        self.conv1 = ConvBlock(in_channels, 64)
        self.conv2 = ConvBlock(64, 128, pool=True) # out_dim : 128 x 112 x 112
        self.res1 = nn.Sequential(ConvBlock(128, 128), ConvBlock(128, 128))

        self.conv3 = ConvBlock(128, 256, pool=True) # out_dim : 256 x 56 x 56
        self.conv4 = ConvBlock(256, 512, pool=True) # out_dim : 512 x 28 x 28
        self.res2 = nn.Sequential(ConvBlock(512, 512), ConvBlock(512, 512))

        self.classifier = nn.Sequential(
            nn.AdaptiveAvgPool2d(1), # ?Όμ²? λ§? ?¬κΈ°λ?? 1x1λ‘? μ‘°μ 
            nn.Flatten(),
            nn.Linear(512, 1)  # ??? ?€?κ°μ ?μΈ?
        )

    def forward(self, xb): # xb? λ‘λ? λ°°μΉ
        out = self.conv1(xb)
        out = self.conv2(out)
        out = self.res1(out) + out
        out = self.conv3(out)
        out = self.conv4(out)
        out = self.res2(out) + out
        out = self.classifier(out)
        return out

#################################################################
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
    
    # Apply camera calibration
    h, w = inputImage.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    inputImage = cv2.undistort(inputImage, mtx, dist, None, newcameramtx)
    x, y, w, h = roi
    
    return inputImage[y:y+h, x:x+w]

def perspectiveWarp(inputImage):

    # Get image size
    img_size = (640, 480)
    
    # Perspective points to be warped
    src = np.float32([[210, 108],
                      [325, 108],
                      [95, 315],
                      [419, 312]])

    # Window to be shown
    dst = np.float32([[130, 194],
                      [510, 194],
                      [130, 460],
                      [510, 460]])

    # Matrix to warp the image for birdseye window
    matrix = cv2.getPerspectiveTransform(src, dst)
    # Inverse matrix to unwarp the image for final window
    minv = cv2.getPerspectiveTransform(dst, src)
    birdseye = cv2.warpPerspective(inputImage, matrix, img_size)

    # Display birdseye view image
    # cv2.imshow("Birdseye" , birdseye)

    return birdseye, minv

def processImage(inputImage):
    
    hsv = cv2.cvtColor(inputImage, cv2.COLOR_BGR2HSV)
    
    lower = np.array([70, 0, 190])
    upper = np.array([255, 255, 255])

    mask = cv2.inRange(hsv, lower, upper)
    mask_3channel = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
    
    return mask_3channel


class Resnet_Pilot:
    def __init__(self):
        self.bridge = CvBridge()
        self.image = None
        self.image_sub = rospy.Subscriber('/camera/image', Image, self.image_callback)
        self.traffic_light_sub = rospy.Subscriber('traffic_light', String, self.traffic_light_callback)
        self.object_detected_sub = rospy.Subscriber('object_detected', String, self.object_detected_callback)
        
        self.pub = rospy.Publisher("control_input", Float32MultiArray, queue_size=10)
        
        self.stage = ['waiting_first', 'waiting_stop', 'waiting_go']
        self.current_stage = self.stage[0]
        
        self.forward = 0.5
        self.backward = 1.0
        self.angle = 0.0

        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = ResNet9(in_channels=1).to(self.device)
        self.model_load_path = '/home/fmtc/catkin_ws/model_20240720_184349.pth'
        self.load_model()

        self.transform = transforms.Compose([
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Grayscale(num_output_channels=1)
            # transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])
        
    
    def load_model(self):
        self.model.load_state_dict(torch.load(self.model_load_path, map_location=self.device))
        self.model.eval()
        print(f"Model loaded from {self.model_load_path}")

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def traffic_light_callback(self, msg):
        rospy.loginfo(f"Received traffic light data: {msg.data}")
        if self.current_stage == 'waiting_stop' and msg.data == 'stop':
            self.forward = 1.0
            self.current_stage = 'waiting_go'
        elif self.current_stage == 'waiting_go' and msg.data == 'go':
            self.forward = 0.0  # ?Ή?  κ°μΌλ‘? ?€? ??¬ μ§μ§
        self.PublishControlInput()

    def object_detected_callback(self, msg):
        rospy.loginfo(f"Received object detected data: {msg.data}")
        if msg.data == "Object detected":
            if self.current_stage == 'waiting_first':
                self.move_to_first_lane()
                self.current_stage = 'waiting_stop'
        self.PublishControlInput()

    def timed_publish(self, angle, duration):
        self.angle = angle
        end_time = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < end_time:
            self.PublishControlInput()
            rospy.sleep(0.1)

    def move_to_first_lane(self):
        self.timed_publish(1.0, 2.0)  # ?Όμͺ½μΌλ‘? 2μ΄κ° μ‘°ν₯
        self.timed_publish(-1.0, 2.0)  # ?€λ₯Έμͺ½?Όλ‘? 2μ΄κ° μ‘°ν₯
        self.timed_publish(0.0, 3.0)  # μ§μ§ 3μ΄?
        self.timed_publish(-1.0, 2.0)  # ?€λ₯Έμͺ½?Όλ‘? 2μ΄κ° μ‘°ν₯
        self.timed_publish(1.0, 2.0)  # ?Όμͺ½μΌλ‘? 2μ΄κ° μ‘°ν₯
        self.timed_publish(0.0, 0.1)  # μ§μ§ 0.1μ΄?
    
    def PublishControlInput(self):
        msg = Float32MultiArray()
        msg.data = [self.forward, self.backward, self.angle]
        self.pub.publish(msg)

    def ModelWork(self):
        if self.image is not None:
            image_calibrated = Calibrate(self.image)
            image_birdseye, _ = perspectiveWarp(image_calibrated)
            image_transformed = processImage(image_birdseye)
            pil_image = PILImage.fromarray(image_transformed)
            image_tensor = self.transform(pil_image).unsqueeze(0).to(self.device)

            with torch.no_grad():
                pred = self.model(image_tensor)
                self.angle = pred.item()
                self.PublishControlInput()

if __name__ == "__main__":
    rospy.init_node('resnet_pilot')
    pilot = Resnet_Pilot()
    pilot.ModelWork()
    rospy.spin()

#if __name__ == '__main__':
#    rospy.init_node('Resnet_Pilot', anonymous=True)
#    rate = rospy.Rate(100)
#    Pilot = Resnet_Pilot()
    
#    while not rospy.is_shutdown():
#        Pilot.ModelWork()
#        Pilot.PublishControlInput()
#        rate.sleep()
