#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from proje.srv import DrawTriangle

rospy.wait_for_service("triangle_service")
try:
    length =float(input("Uzunluk giriniz : "))
    service = rospy.ServiceProxy("triangle_service",DrawTriangle)
    service(length)
except rospy.ServiceException:
    print("Servis hatasi")