#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#Gereken kütüphaneler
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Lidar:
    def __init__(self):
        # ROS düğümü
        rospy.init_node('lidar_node')
        
        # Hız komutlarını yayınlamak için
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        
        # Hız mesajını
        self.speed_message = Twist()
        
        # Lazer verilerini dinlemek için
        rospy.Subscriber('scan', LaserScan, self.lazerCallback)
        
        # ROS düğümünün sürekli çalışmasını sağlar
        rospy.spin()
        
    def lazerCallback(self, msg):
        # Ön kısımı tanımlamak için
        fl = list(msg.ranges[0:9])
        fr = list(msg.ranges[350:359])
        front = fr + fl
        
        # Minimum mesafe
        mf = min(front)
        
        # Mesafe bilgisi
        rospy.loginfo("Minimum Mesafe: %.2f metere", mf)
        
        
        if mf < 1.0:
            # Eğer minimum mesafe 1.0 metreden küçükse, durur
            self.speed_message.linear.x = 0.0
        else:
            #Değilse devam eder
            self.speed_message.linear.x = 0.5
        # Hız komutu
        self.pub.publish(self.speed_message)
Lidar()

