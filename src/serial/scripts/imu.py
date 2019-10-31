import rospy
import serial



imu = serial.Serial('/dev/ttyUSB0', 19200)

while True:
	imu.readline()
	imu.flush()
	for data in imu.readline():
		imu_str = data.strip("#YPR")
		imu_str = imu_str.strip("/r/n")
		imu_str = imu_str.split(",")
