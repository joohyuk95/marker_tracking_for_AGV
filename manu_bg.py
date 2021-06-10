#!/usr/bin/env python
import rospy
from serial import Serial

ser = Serial('/dev/ttyMANU', 115200)

str0 = '0'
str1 = '1'

while True:
        pa = rospy.get_param('/tb2/mission')
        if pa == True:
                ser.write(str0.encode())
                rospy.sleep(2)
                ser.write(str1.encode())
                break