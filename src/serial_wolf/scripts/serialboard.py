#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
from serial.msg import ThrusterCmd

import serial


rospy.init_node('serial_board')
rospy.Subscriber("thruster_values", ThrusterCmd, set_thrusters)
ser = serial.Serial("/dev/ttyUSB0", baudrate=57600)
killed = False


def get_killed():
    if ser.inWaiting() > 0:
	ser.read(ser.inWaiting())
	ser.write(bytearray([0x7e, 0xff, 0x30]))
	return ord(ser.read(1)[0])
    
def set_thrusters(data):
    if not get_killed():
	ser.write(bytearray([0x7e, 0xff, 0x10, abs(data[0]), abs(data[1]), abs(data[2]), abs(data[3])]))
	ser.write(bytearray([0x7e, 0xff, 0x12, sign(data[0]), sign(data[1]), sign(data[2]), sign(data[3])]))
	ser.write(bytearray([0x7e, 0xff, 0x11, sign(data[4]), abs(data[4])]))
