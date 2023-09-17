#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Gereken kütüphaneler
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class Camera():
    def __init__(self):
        # ROS düğümü başlatma
        rospy.init_node('camera_node')
        self.bridge = CvBridge()
        
        # Kameradan görüntü verisi almak için abone olma
        rospy.Subscriber("camera/rgb/image_raw", Image, self.cameraCallback)
        
        # Sonsuz döngüyü sürdürme
        rospy.spin()

    def cameraCallback(self, msg):
        # ROS mesajını OpenCV görüntüsüne çevirme
        img = self.bridge.imgmsg_to_cv2(msg, "mono8")
        img2 = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Görüntüyü işleme ekleme
        ret, threshold = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY_INV)
        countours, hierarchy = cv2.findContours(threshold, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        #Kenar çizme ve ağırlık merkezi bulma
        cv2.drawContours(img2, countours, -1, (0, 0, 255), 5)
        cnt = countours[0]
        M = cv2.moments(cnt)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        cv2.circle(img2, (cx, cy), 5, (255, 255, 255), -1)

        # Görüntüyü paneli
        cv2.imshow("robocam", img2)
        cv2.waitKey(1)

Camera()
