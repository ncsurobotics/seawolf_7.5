#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int64
from std_msgs.msg import Bool
from serial_wolf.msg import ThrusterCmd

import serial
import time

def sign(val):
	return val > 0

def get_killed():
    if ser.inWaiting() > 0:
	ser.read(ser.inWaiting())
        
    ser.write(bytearray([0x7e, 0xff, 0x30]))
    return ord(ser.read(1)[0])

def get_depth():
    if ser.inWaiting() > 0:
	ser.read(ser.inWaiting())
    ser.write(bytearray([0x7e, 0xff, 0x20]))
    #print("Written")
    a = ser.read(2)
    #print(a[0], a[1])
    return ord(a[1])
            
def set_thrusters(data):
    if not killed:
	ser.write(bytearray([0x7e, 0xff, 0x10, abs(data[0]), abs(data[1]), abs(data[2]), abs(data[3])]))
	ser.write(bytearray([0x7e, 0xff, 0x12, sign(data[0]), sign(data[1]), sign(data[2]), sign(data[3])]))
	ser.write(bytearray([0x7e, 0xff, 0x11, sign(data[4]), abs(data[4])]))


                                        
rospy.init_node('serial_board')
rospy.Subscriber("thruster_values", ThrusterCmd, set_thrusters)
killed = False
depth = 0
ser = serial.Serial("/dev/ttyUSB1", baudrate=57600)
armedPub = rospy.Publisher("robot_status", Bool, queue_size=3)
depthPub = rospy.Publisher("depth_data", Int64, queue_size=3)
time.sleep(1)
rate = rospy.Rate(10)

while True:
    set_thrusters([0x80, 0x80, 0x80, 0x80, 0x80])
    killed = get_killed()
    depth =  get_depth()
    armedPub.publish(killed)
    depthPub.publish(depth)
    rate.sleep()
