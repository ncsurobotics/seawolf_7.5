# !/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from serial.msg import ThrusterCmd

#z speed from depth sensor
#some interpolation from commanded vel to thruster percent output

max_vel = 0.5  #arbitrary m/s val


def cmd_cb(data):
  x = data.linear.x
  y = data.linear.y
  z = data.linear.z
  
  r = data.angular.x
  p = data.angular.y
  y = data.angular.z

  thrusterCmd = Float64MultiArray()
  thrusterCmd.data = []
  thrusterCmd.data.append(788888.0)
  thrusterPub.publish(thrusterCmd)`

#Initialize ROS node
rospy.init_node("twist_to_thrusters")

#ROS Subscribers
cmdSub = rospy.Subscriber("thruster_cmd_vel", Twist, cmd_cb)

#ROS Publishers
thrusterPub = rospy.Publisher("thruster_values", Float64MultiArray, queue_size=3)
