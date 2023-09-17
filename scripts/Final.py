#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#Gereken Kütüphaneler
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

class ObjectDetection():
    def __init__(self):
         # ROS düğümünü başlat
        rospy.init_node("object_detection") 
        
        # Hız komutları yayınlamak için
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.speed_message = Twist()
        self.bridge = CvBridge()
        self.min_front = 0

        # Görüntü ve LIDAR verilerini dinlemek için aboneleri başlatın
        rospy.Subscriber("camera/rgb/image_raw", Image, self.camera_callback)
        rospy.Subscriber("scan", LaserScan, self.laser_callback)
        rospy.spin()

    def filter_color(self, gray_image, lower_color, upper_color):
        # Gri görüntü üzerinde belirli bir renge göre maske oluşturun
        mask = cv2.inRange(gray_image, lower_color, upper_color)
        return mask

    def center_of_gravity(self, mask):
        # Maske üzerindeki en büyük konturu bulun ve geometri merkezini hesaplayın
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            
            if M['m00'] > 0:
                x = int(M['m10'] / M['m00'])
                y = int(M['m01'] / M['m00'])
                return x, y
        
        return None, None

    def camera_callback(self, request):
        # Kamera verilerini işlemek için
        image = self.bridge.imgmsg_to_cv2(request, "bgr8")
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        lower_gray = np.array([30], dtype="uint8")
        upper_gray = np.array([150], dtype="uint8")
        mask = self.filter_color(gray, lower_gray, upper_gray)

        h, w = image.shape[:2]
        x, y = self.center_of_gravity(mask)

        if x is not None and y is not None:
            deviation = x - w / 2

            if self.min_front > 1.0:
                self.speed_message.linear.x = 0.25
                self.speed_message.angular.z = -deviation / 100
                self.pub.publish(self.speed_message)
            elif self.min_front < 1.0:
                self.speed_message.linear.x = 0.0
                self.speed_message.angular.z = 0.0
                self.pub.publish(self.speed_message)

            cv2.circle(image, (x, y), 5, (255, 0, 0), -1)
        else:
            self.speed_message.linear.x = 0.0
            self.speed_message.angular.z = 0.5
            self.pub.publish(self.speed_message)
        #Görüntü paneli
        cv2.imshow("image", image)
        cv2.imshow("gray_image", mask)
        cv2.waitKey(1)

    def laser_callback(self, request):
        # LIDAR işlemleri
        right_front = list(request.ranges[0:9])
        left_front = list(request.ranges[350:359])
        front = right_front + left_front
        left = list(request.ranges[80:100])
        right = list(request.ranges[260:280])
        back = list(request.ranges[170:190])
        
        min_left = min(left)
        min_right = min(right)
        min_back = min(back)
        
        self.min_front = min(front)
        print(min_left, min_right, self.min_front, min_back)

ObjectDetection()
