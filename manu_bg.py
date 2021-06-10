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
    if rospy.get_param('/'+tb_id+'/mission_start') == True and rospy.get_param('/'+tb_id+'/mission') == True:
        ser.write(str0.encode())
        rospy.sleep(2)
        ser.write(str1.encode())
        break

while True:
    if rospy.get_param('/'+tb_id+'/mission_finished') == True and rospy.get_param('/'+tb_id+'/mission') == False:
        ser.write(str1.encode())
        rospy.sleep(2)
        ser.write(str0.encode())
        break
