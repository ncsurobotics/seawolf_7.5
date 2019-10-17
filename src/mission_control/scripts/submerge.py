#ROS imports
import rospy
from geometry_msgs.msg import Vector3
from serial.msg import ThrusterData
from serial.msg import ThrusterCmd
from geometry_msgs inmport Twist
from std_msgs import Float64

Kp = 0
Ki = 0
Kd = 0

last_error = 0
error_integral = 0
error_derivative = 0

last_thruster_msg = 0
thruster_vals = []
depth = 0

#Define callback function for thruster data
def thruster_cb(data):
  thruster_vals = data.thruster_values 
  depth = data.depth
  last_thruster_msg = header.stamp.to_sec()

last_imu_msg == 0
euler_angle = []

#Define callback function for imu data
def imu_cb(data):
  euler_angle[0] = data.x
  euler_angle[1] = data.y
  euler_angle[2] = data.z
  last_imu_msg = header.stamp.to_sec()

depth_target = 0
def depth_target_cb(data):
  if not data.data == depth_target:
    error_integral = 0
  depth_target = data.data

#Initialize ROS node called submerge_mission
rospy.init_node("submerge_mission")

#ROS Subscribers
thrusterSub = rospy.Subscriber("thruster_sub", ThrusterData, thruster_cb)
imuSub = rospy.Subscriber("imu_sub", Vector3, imu_cb)
targetDepthSub = rospy.Subscriber("depth_target", Float64, depth_target_cb)

#ROS Publisher
thrusterPub = rospy.Publisher("thruster_pub", ThrusterCmd, queue_size=3)

#Called to tell subscribers to constantly look for msgs
rospy.spin()

loop_rate = rospy.Rate(10) # 10 Hzdd
last_time = rospy.Time.now().to_sec()

#Loop through PID code until ROS is shutdown
while not rospy.is_shutdown():
  cur_time = rospy.Time.now().to_sec()
  dt = cur_time - last_time
  if(dt < 0.001):
    loop_rate.sleep()
    continue

  thrusterMsg = Twist()

  error = target_depth - depth
  error_integral += error*dt
  error_derivative = (error - last_error) / dt

  output = Kp*error + Ki*error_integral + Kd*error_derivative

  thrusterMsg.linear = [0, 0, output]

  last_error = error
  loop_rate.sleep()
