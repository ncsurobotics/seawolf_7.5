#! /usr/bin/env python
#ROS imports
import rospy
from geometry_msgs.msg import Vector3
from serial.msg import ThrusterData
from serial.msg import ThrusterCmd
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import numpy as np

depth_pid = [1, 0, 0]
rpy_pid = [0, 0, 0]

last_depth_error = 0
depth_error_integral = 0
depth_error_derivative = 0

last_rpy_error = [0,0,0]
rpy_error_integral = [0,0,0]
rpy_error_derivative = [0,0,0]

#Define callback function for thruster data
thruster_vals = []
depth = 0
def thruster_cb(data):
  global thruster_vals, depth
  thruster_vals = data.thruster_values 
  depth = data.depth

#Define callback function for setting depth target
target_depth = 0
def target_depth_cb(data):
  global target_depth, depth_error_integral
  if not data.data == target_depth:
    depth_error_integral = 0
  target_depth = data.data

#Define callback function for imu data
rpy = np.array([0,0,0])
def imu_cb(data):
  rpy[0] = data.x
  rpy[1] = data.y
  rpy[2] = data.z

#Define callback function for setting orientation target
target_rpy = np.array([0,0,0])
def orientation_target_cb(data):
  target_rpy[0] = data.x
  target_rpy[1] = data.y
  target_rpy[2] = data.z

#Initialize ROS node called submerge_mission
rospy.init_node("submerge_mission")

#ROS Subscribers
thrusterSub = rospy.Subscriber("thruster_sub", ThrusterData, thruster_cb)
targetDepthSub = rospy.Subscriber("target_depth", Float64, target_depth_cb)
imuSub = rospy.Subscriber("imu_sub", Vector3, imu_cb)
targetOrientationSub = rospy.Subscriber("orientation_target", Vector3, orientation_target_cb)

#ROS Publisher
thrusterPub = rospy.Publisher("thruster_pub", Twist, queue_size=3)

#Called to tell subscribers to constantly look for msgs

loop_rate = rospy.Rate(10) # 10 Hz
last_time = rospy.Time.now().to_sec()

#Loop through PID code until ROS is shutdown
while not rospy.is_shutdown():
  cur_time = rospy.Time.now().to_sec()
  dt = cur_time - last_time
  if(dt < 0.001):
    loop_rate.sleep()
    continue

  thrusterMsg = Twist()

  depth_error = target_depth - depth
  depth_error_integral += depth_error*dt
  depth_error_derivative = (depth_error - last_depth_error) / dt

  rpy_error = target_rpy - rpy
  rpy_error_integral += rpy_error*dt
  rpy_error_derivative = (rpy_error - last_rpy_error) / dt

  z_output = depth_pid[0]*depth_error + depth_pid[1]*depth_error_integral + depth_pid[2]*depth_error_derivative
  rpy_output = rpy_pid[0]*rpy_error + rpy_pid[1]*rpy_error_integral + rpy_pid[2]*rpy_error_derivative

  thrusterMsg.linear.x = 0
  thrusterMsg.linear.y = 0
  thrusterMsg.linear.y = z_output
  thrusterMsg.angular.x = rpy_output[0]
  thrusterMsg.angular.y = rpy_output[1]
  thrusterMsg.angular.z = rpy_output[2]
  thrusterPub.publish(thrusterMsg)

  last_depth_error = depth_error
  last_rpy_error = rpy_error
  loop_rate.sleep()
