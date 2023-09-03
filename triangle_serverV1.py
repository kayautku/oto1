#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from proje.srv import DrawTriangle

def triangleFunction(request):
    velocity_msg = Twist()
    linear_velocity = 0.5
    velocity_msg.linear.x = linear_velocity
    length = request.length
    t0 = rospy.Time.now().to_sec()
    movement = 0
    angle = 0
    for i in range(0, 3):
        velocity_msg.linear.x = 5
        while (length > movement):
            pub.publish(velocity_msg)
            t1 = rospy.Time.now().to_sec()
            movement = linear_velocity * (t1 - t0)
        velocity_msg.linear.x = 0.0
        pub.publish(velocity_msg)
        movement = 0

        t0 = rospy.Time.now().to_sec()
        while (angle < 2.0944):  # 120 dereceye eşdeğer 2.0944 radyan
            angle = 0
            angular_velocity_msg = Twist()  
            angular_velocity = 1.0  
            angular_velocity_msg.angular.z = angular_velocity 
            pub.publish(angular_velocity_msg)
            t1 = rospy.Time.now().to_sec()
            angular_velocity = 1
            angle = angular_velocity * (t1 - t0)
        angular_velocity_msg.angular.z = 0.0
        t0 = rospy.Time.now().to_sec()
        pub.publish(angular_velocity_msg)
        angle = 0
rospy.init_node("draw_triangle")
pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
rospy.Service("triangle_service", DrawTriangle, triangleFunction)
rospy.loginfo("Servis hazir, üçgen çizimi için bekleniyor...")
rospy.spin()