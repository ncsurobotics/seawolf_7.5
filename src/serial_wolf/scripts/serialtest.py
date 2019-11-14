import rospy
import serial
from threading import Thread

def sign(val):
	return val > 0


class Thrusters:
	
	def __init__(self, port):
		self.ser = serial.Serial(port, baudrate=57600)
		self.read_thread = Thread(target=self.read_thread)
		#self.read_thread.start()
		self.thrusters = []
		self.killed = False
		self.close = False
	def resetAll(self):
		self.ser.write(bytearray([0x7e, 0xff, 0x80]))


	def kill(self):
		self.close = True
		self.ser.close()

	def setThrusters(self, thrusters):
		if not self.killed:
			self.thrusters = thrusters
			self.ser.write(bytearray([0x7e, 0xff, 0x10, abs(thrusters[0]), abs(thrusters[1]), abs(thrusters[2]), abs(thrusters[3])]))
			self.ser.write(bytearray([0x7e, 0xff, 0x12, sign(thrusters[0]), sign(thrusters[1]), sign(thrusters[2]), sign(thrusters[3])]))
			self.ser.write(bytearray([0x7e, 0xff, 0x11, sign(thrusters[4]), abs(thrusters[4])]))

	def getDepth(self):
		if self.ser.inWaiting() > 0:
			self.ser.read(self.ser.inWaiting())
		self.ser.write(bytearray([0x7e, 0xff, 0x20]))
		#print("Written")
		a = self.ser.read(2)
		#print(a[0], a[1])
		return a[1]
	def getKill(self):
		if self.ser.inWaiting() > 0:
			self.ser.read(self.ser.inWaiting())
		self.ser.write(bytearray([0x7e, 0xff, 0x30]))
		return ord(self.ser.read(1)[0])

	def read_thread(self):
		self.ser.read(self.ser.inWaiting())
		if self.close:
			return
		while True:
			while self.ser.inWaiting() > 0:
				if self.close:
					return
				byte = self.ser.read(1)[0]
				if ord(byte) == 1:
					print("Killed")
					self.killed = True
					self.resetAll()
				elif ord(byte) == 0x0:
					self.killed = False
					print("Un-killed")
				elif ord(byte) == 0x7e:
					print(self.ser.read(2))

thrusters = Thrusters("/dev/ttyUSB1")
thrusters.read_thread()
