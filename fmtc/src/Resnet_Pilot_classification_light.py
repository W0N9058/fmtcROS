#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32, Bool
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import pickle
import cv2
import numpy as np
import os
import torch
import torch.nn as nn
import torch.nn.functional as F
from torchvision import transforms
from PIL import Image as PILImage

#################################### 모델 선언 ##############################

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
        return self.relu2(out) + x # ReLU는 입력을 더한 후 또는 이전에 적용할 수 있습니다.


class ImageRegressionBase(nn.Module):
    def training_step(self, batch):
        images, labels = batch
        labels = labels.unsqueeze(1).float()  # 레이블을 (batch_size, 1) 형태로 변환하고 Float 타입으로 변경
        out = self(images)                  # 예측 생성
        loss = nn.CrossEntropyLoss()      # 손실 계산 (회귀를 위한 평균 제곱 오차)
        return loss(out, labels)

    def validation_step(self, batch):
        images, labels = batch
        labels = labels.unsqueeze(1).float()  # 레이블을 (batch_size, 1) 형태로 변환하고 Float 타입으로 변경
        out = self(images)                   # 예측 생성
        loss = nn.CrossEntropyLoss()      # 손실 계산
        pred = out.max(1, keepdim=True)[1]
        correct = pred.eq(labels.view_as(pred)).sum().item()
        return {"val_loss": loss(out, labels).detach(), "val_correct": correct}

    def validation_epoch_end(self, outputs):
        batch_losses = [x["val_loss"] for x in outputs]
        epoch_loss = torch.stack(batch_losses).mean()       # 손실 결합
        return {"val_loss": epoch_loss} # 손실 결합

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

        self.conv1 = ConvBlock(in_channels, 64, pool=True) # out_dim : 64 x 112 x 112
        self.res1 = nn.Sequential(ConvBlock(64, 64), ConvBlock(64, 64))

        self.conv3 = ConvBlock(64, 128, pool=True)  # out_dim : 128 x 56 x 56
        self.res2 = nn.Sequential(ConvBlock(128, 128), ConvBlock(128, 128))

        self.classifier = nn.Sequential(
            nn.AdaptiveAvgPool2d(1), # 피처 맵 크기를 1x1로 조정
            nn.Flatten(),
            nn.Linear(128, 9)  # 하나의 실수값을 예측
        )

    def forward(self, xb): # xb는 로드된 배치
        out = self.conv1(xb)
        out = self.res1(out) + out
        out = self.conv3(out)
        out = self.res2(out) + out
        out = self.classifier(out)
        return out

################################################################

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
    dst = np.float32([[130, 50],
                      [510, 50],
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
    
    lower = np.array([70, 0, 230])
    upper = np.array([255, 255, 255])

    mask = cv2.inRange(hsv, lower, upper)
    mask_3channel = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
    
    return mask_3channel

class Resnet_Pilot:
    def __init__(self):
        self.bridge = CvBridge()
        self.image = None
        # 카메라 정보 받아오기
        self.image_sub = rospy.Subscriber('/camera/image', Image, self.image_callback)
        # 조향값
        self.pub = rospy.Publisher("control_input", Float32MultiArray, queue_size=10)
        
        self.forward  = -1.0
        self.backward = 1.0
        self.angle    = 0.0

        # 모델 로드
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = ResNet9(in_channels=1).to(self.device)
        self.model_load_path = '/home/fmtc/catkin_ws/model_20240720_184349.pth'
        self.load_model()

        # 전처리 정의
        self.transform = transforms.Compose([
            transforms.Resize((224, 224)),  # ResNet 입력 크기에 맞게 조정
            transforms.ToTensor(),
            transforms.Grayscale(num_output_channels=1)# 텐서로 변환
            # transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])  # ImageNet 데이터셋의 평균과 표준편차로 정규화
        ])

        self.angle_label = np.linspace(-1.0, 1.0, 9)

    def load_model(self):
        self.model.load_state_dict(torch.load(self.model_load_path, map_location=self.device))
        self.model.eval()  # 평가 모드로 설정
        print(f"Model loaded from {self.model_load_path}")

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    
    def PublishControlInput(self):
        msg = Float32MultiArray()
        msg.data = [self.forward, self.backward, self.angle]
        self.pub.publish(msg)
    
    def ModelWork(self):
        if self.image is not None:
            image_calibrated = Calibrate(self.image)
            image_birdseye, _ = perspectiveWarp(image_calibrated)
            image_transformed = processImage(image_birdseye)
            cv2.imshow("lane detection", image_transformed)
            pil_image = PILImage.fromarray(image_transformed)
            image_tensor = self.transform(pil_image).unsqueeze(0).to(self.device)

            with torch.no_grad():
                out = self.model(image_tensor)
                pred = out.max(1, keepdim=True)[1]
                # self.angle = (pred.item()-482.5) * 2 / (735*0.8)  # 예측값을 조향값으로 설정
                self.angle = self.angle_label[pred.item()]
                print(f"angle: {self.angle}")
                
                
if __name__ == '__main__':
    rospy.init_node('Resnet_Pilot', anonymous=True)
    rate = rospy.Rate(100)
    Pilot = Resnet_Pilot()
    
    while not rospy.is_shutdown():
        Pilot.ModelWork()
        Pilot.PublishControlInput()
        rate.sleep()

