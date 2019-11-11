#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3
import serial

def readImuTalker():
    rospy.init_node('IMU')
    pub = rospy.Publisher('imu_data', Vector3, queue_size=10)
    # rate = rospy.Rate(10) # 10hz #i don't know if want this in here but I kept it just in case
    while not rospy.is_shutdown():
        imu = serial.Serial("/dev/ttyUSB0", baudrate=57600)
        imu.readline()
        imu.flush()
        while True:
            imu.readline()
            for str in imu:
                imu_str = str.strip("#YPR=")
                imu_str = imu_str.strip("\r\n")
                imu_str = imu_str.split(",")
                imu_data = Vector3()
                imu_data.x = float(imu_str[0])
                imu_data.y = float(imu_str[1])
                imu_data.z = float(imu_str[2])
                #rospy.loginfo(imu_str)  # should log the serial data retrieved
                pub.publish(imu_data)  # should print the serial data retrieved
            # rate.sleep() #again kept just in case

readImuTalker()
