#!/usr/bin/env python

import sys
import rospy
from serial import Serial

if len(sys.argv) != 2:
        print("Usage: python manu_bg.py [tb2|tb3]")
        sys.exit()

ser = Serial('/dev/ttyMANU', 115200)

str0 = '0'
str1 = '1'

tb_id = sys.argv[1]

while True:
        pa = rospy.get_param('/'+tb_id+'/mission')

        if pa == True:
                ser.write(str0.encode())
                rospy.sleep(2)
                ser.write(str1.encode())
                break