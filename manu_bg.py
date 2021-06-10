#!/usr/bin/env python

import rospy
from serial import Serial

ser = Serial('/dev/ttyMANU', 115200)

str0 = '0'
str1 = '1'

while True:
        tb_id = rospy.get_param('/tb_id')
        pa = rospy.get_param('/'+tb_id+'/mission')
        if pa == True:
                ser.write(str0.encode())
                rospy.sleep(2)
                ser.write(str1.encode())
                break