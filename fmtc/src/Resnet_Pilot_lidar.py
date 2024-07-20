#!/usr/bin/env python

import rospy
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
from threading import Lock

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
        loss = F.mse_loss(out, labels)      # 손실 계산 (회귀를 위한 평균 제곱 오차)
        return loss

    def validation_step(self, batch):
        images, labels = batch
        labels = labels.unsqueeze(1).float()  # 레이블을 (batch_size, 1) 형태로 변환하고 Float 타입으로 변경
        out = self(images)                   # 예측 생성
        loss = F.mse_loss(out, labels)       # 손실 계산
        return {"val_loss": loss.detach()}

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

        self.conv1 = ConvBlock(in_channels, 64)
        self.conv2 = ConvBlock(64, 128, pool=True) # out_dim : 128 x 112 x 112
        self.res1 = nn.Sequential(ConvBlock(128, 128), ConvBlock(128, 128))

        self.conv3 = ConvBlock(128, 256, pool=True) # out_dim : 256 x 56 x 56
        self.conv4 = ConvBlock(256, 512, pool=True) # out_dim : 512 x 28 x 28
        self.res2 = nn.Sequential(ConvBlock(512, 512), ConvBlock(512, 512))

        self.classifier = nn.Sequential(
            nn.AdaptiveAvgPool2d(1), # 피처 맵 크기를 1x1로 조정
            nn.Flatten(),
            nn.Linear(512, 1)  # 하나의 실수값을 예측
        )

    def forward(self, xb): # xb는 로드된 배치
        out = self.conv1(xb)
        out = self.conv2(out)
        out = self.res1(out) + out
        out = self.conv3(out)
        out = self.conv4(out)
        out = self.res2(out) + out
        out = self.classifier(out)
        return out

#################################################################

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
        self.model = ResNet9(in_channels=3).to(self.device)
        self.model_load_path = '/home/fmtc/catkin_ws/model.pth'
        self.load_model()

        self.transform = transforms.Compose([
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])
        
        self.lock = Lock()  # Lock to manage the execution of critical sections

    def load_model(self):
        self.model.load_state_dict(torch.load(self.model_load_path, map_location=self.device))
        self.model.eval()
        print(f"Model loaded from {self.model_load_path}")

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def traffic_light_callback(self, msg):
        with self.lock:  # Acquire lock to block ModelWork
            rospy.loginfo(f"Received traffic light data: {msg.data}")
            if self.current_stage == 'waiting_stop' and msg.data == 'stop':
                self.forward = 1.0
                self.current_stage = 'waiting_go'
            elif self.current_stage == 'waiting_go' and msg.data == 'go':
                self.forward = 0.0  # 특정 값으로 설정하여 직진
                self.PublishControlInput()

    def object_detected_callback(self, msg):
        with self.lock:  # Acquire lock to block ModelWork
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
        self.timed_publish(1.0, 2.0)  # 왼쪽으로 2초간 조향
        self.timed_publish(-1.0, 2.0)  # 오른쪽으로 2초간 조향
        self.timed_publish(0.0, 3.0)  # 직진 3초
        self.timed_publish(-1.0, 2.0)  # 오른쪽으로 2초간 조향
        self.timed_publish(1.0, 2.0)  # 왼쪽으로 2초간 조향
        self.timed_publish(0.0, 0.1)  # 직진 0.1초
    
    def PublishControlInput(self):
        msg = Float32MultiArray()
        msg.data = [self.forward, self.backward, self.angle]
        self.pub.publish(msg)

    def ModelWork(self):
        while not rospy.is_shutdown():
            with self.lock:  # Acquire lock to ensure this block is not executed during critical sections
                if self.image is not None:
                    image_rgb = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
                    pil_image = PILImage.fromarray(image_rgb)
                    image_tensor = self.transform(pil_image).unsqueeze(0).to(self.device)

                    with torch.no_grad():
                        pred = self.model(image_tensor)
                        self.angle = (pred.item()-482.5) * 2 / (735*0.8)
                        self.PublishControlInput()
            rospy.sleep(0.1)  # Add a small delay to prevent high CPU usage

    def run(self):
        model_thread = threading.Thread(target=self.ModelWork)
        model_thread.start()
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('resnet_pilot')
    pilot = Resnet_Pilot()
    pilot.run()

