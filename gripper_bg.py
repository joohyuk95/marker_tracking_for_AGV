#!/usr/bin/env python
import sys
import rospy
from serial import Serial

if len(sys.argv) != 2:
        print("Usage: python manu_bg.py [tb2|tb3]")
        sys.exit()

ser = Serial('/dev/ttyGRP', 115200)

str0 = '0'
str1 = '1'

tb_id = sys.argv[1]

while True:
    if rospy.get_param('/'+tb_id+'/mission') == True:
        ser.write(str0.encode())
        ser.write(str0.encode())
        print("open grab")
        rospy.sleep(2)              
        ser.write(str1.encode())
        print("close grab")
        rospy.sleep(1)
        rospy.set_param('/'+tb_id+'/mission_finished', True)
        print("complete")
        break
    else:
        pass

while True:
    if rospy.get_param('/'+tb_id+'/parking_finished') == True :
        ser.write(str0.encode())
        rospy.sleep(1)
        ser.write(str1.encode())
        rospy.sleep(1)
        break
    else:
        pass
