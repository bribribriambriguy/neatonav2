import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import JointState
from .neato_driver import *
import serial
import math
import numpy as np

class neato_node(rclpy.node.Node):

	def __init__(self):
		super().__init__('Neato_node')
  
		self.declare_parameter('base_frame','base_link')
		self.declare_parameter('odom_frame', 'odom')
		self.declare_parameter('neato_port','/dev/ttyACM0')
		self.declare_parameter('wheel_track', 0.240)
		self.declare_parameter('wheel_radius', 0.0381)
		self.declare_parameter('laser_frame','laser')
		self.declare_parameter('max_x_speed', 0.30)
		self.declare_parameter('max_z_speed', 0) # unlimited
		self.declare_parameter('enable_odom', True)
		self.declare_parameter('enable_scan', True)
  
		self.get_logger().info('base_frame: '+ self.get_parameter('base_frame').get_parameter_value()._string_value)
		self.get_logger().info('odom_frame: '+ self.get_parameter('odom_frame').get_parameter_value()._string_value)
		self.get_logger().info('laser_frame: '+ self.get_parameter('laser_frame').get_parameter_value()._string_value)
		self.get_logger().info('neato_port: ' + self.get_parameter('neato_port').get_parameter_value()._string_value)
		self.get_logger().info('wheel_track: '+ str(self.get_parameter('wheel_track').get_parameter_value()._double_value))
		self.get_logger().info('wheel_radius: '+ str(self.get_parameter('wheel_radius').get_parameter_value()._double_value))
		self.get_logger().info('max_x_speed: '+ str(self.get_parameter('max_x_speed').get_parameter_value()._double_value))
		self.get_logger().info('max_z_speed: '+ str(self.get_parameter('max_z_speed').get_parameter_value()._double_value))
		self.get_logger().info('enable_odom: '+ str(self.get_parameter('enable_odom').get_parameter_value().bool_value))
		self.get_logger().info('enable_scan: '+ str(self.get_parameter('enable_scan').get_parameter_value().bool_value))
  
		self.left_wheel_pos_prev = 0
		self.right_wheel_pos_prev = 0
		self.x = 0
		self.y = 0
		self.th  = 0
		self.moving_prev = 0
		self.useOdom = self.get_parameter('enable_odom').get_parameter_value().bool_value
		self.useLaser = self.get_parameter('enable_scan').get_parameter_value().bool_value
		self.delta_left_wheel = 0
		self.delta_right_wheel = 0
		self.cmd_vel = None
		self.max_x_speed = self.get_parameter('max_x_speed').get_parameter_value()._double_value
		self.max_z_speed = self.get_parameter('max_z_speed').get_parameter_value()._double_value
  
		self.start_neato()
  
		if self.useOdom:
			self.init_odom()
   
		if self.useLaser:
			self.init_laser()
   
		self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 1)
	
	def init_joint(self):
		self.wheel_pub = self.create_publisher(JointState, '/joint_states', 10)
 
	def init_odom(self):
		self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
		self.odom_broadcaster = TransformBroadcaster(self)
  
	def init_laser(self):
		self.laser_pub = self.create_publisher(LaserScan, '/scan', 10)
		SetLDSRotation(True)
  
		self.scan = LaserScan()
  
		# set fixed things
		self.scan.angle_min = 0.0
		self.scan.angle_max = 359.0 * math.pi / 180.0
		self.scan.angle_increment = math.pi / 180.0
		self.scan.range_min = 0.016
		self.scan.range_max = 6.0
  
	def start_neato(self):
		try:	
			port = self.get_parameter("neato_port").get_parameter_value().string_value
			init(port,False)
			self.get_logger().info('succesfully established connection with neato on ' + port)
			TestMode(True)
			SetLED(BacklightStatus.On, ButtonColors.Green)
			PlaySound(Sounds.WakingUp)
		except serial.serialutil.SerialException:
			self.get_logger().error('could not open ' + port)
			node.destroy_node()
			rclpy.shutdown()

	def odomPub(self):
		wheel_radius = self.get_parameter('wheel_radius').get_parameter_value()._double_value
		lastTime = self.get_clock().now()
		motors = GetMotors(leftWheel=True, rightWheel=True)
		left_wheel_pos = motors.get("LeftWheel_PositionInMM") / 1000.0
		right_wheel_pos = motors.get("RightWheel_PositionInMM") / 1000.0
  
		# if statement replaces having to set the (right/left)_wheel_pos_prev in the init_odom() in ROS1 neato package by brannonvann as to not have big jumps in position when there was none
		if self.left_wheel_pos_prev != 0 and self.right_wheel_pos_prev != 0 and right_wheel_pos != 0 and left_wheel_pos != 0:
			self.delta_left_wheel = (
				left_wheel_pos - self.left_wheel_pos_prev
			)  # left wheel delta in meters
			self.delta_right_wheel = (
				right_wheel_pos - self.right_wheel_pos_prev
			)  # right wheel delta in meters
		
		delta_time = lastTime - self.get_clock().now() # find delta time to be used in nav_msgs/Odometry message
		self.left_wheel_pos_prev = left_wheel_pos
		self.right_wheel_pos_prev = right_wheel_pos
  
		wheel_msg = JointState()
		wheel_msg.header.stamp = self.get_clock().now().to_msg()
		wheel_msg.name[0] = "wheel_left_joint"
		wheel_msg.name[1] = "wheel_right_joint"
		wheel_msg.position[0] = left_wheel_pos/wheel_radius
		wheel_msg.position[1] = right_wheel_pos/wheel_radius

		self.wheel_pub.publish(wheel_msg)
  
		ds = (
			self.delta_left_wheel + self.delta_right_wheel
		) / 2.0
  
		dth = math.atan2(self.delta_right_wheel - self.delta_left_wheel, self.get_parameter('wheel_track').get_parameter_value()._double_value)  # turn angle
  
		self.x += ds * math.cos(self.th + dth / 2.0)
		self.y += ds * math.sin(self.th + dth / 2.0)
		self.th += dth

		# setup TransformStamped() that will broadcast (default: odom -> base_link)
		t = TransformStamped()

		# setup header
		t.header.stamp = self.get_clock().now().to_msg()
		t.header.frame_id = self.get_parameter('odom_frame').get_parameter_value().string_value
		t.child_frame_id = self.get_parameter('base_frame').get_parameter_value().string_value

		# initialize transform
		t.transform.translation.x = self.x
		t.transform.translation.y = self.y
		t.transform.translation.z = 0.0

		q = quaternion_from_euler(0, 0, self.th) # convert eular to quaternion rotation
		t.transform.rotation.x = q[0]
		t.transform.rotation.y = q[1]
		t.transform.rotation.z = q[2]
		t.transform.rotation.w = q[3]
  
		self.odom_broadcaster.sendTransform(t) # broadcast transform
  
		# setup Odometry message
		odom = Odometry()

		#setup header
		odom.header.stamp = self.get_clock().now().to_msg()
		odom.header.frame_id = self.get_parameter('odom_frame').get_parameter_value().string_value
		odom.child_frame_id = self.get_parameter('base_frame').get_parameter_value().string_value
	
		# initialize Odometry message
		odom.pose.pose.position.x = self.x
		odom.pose.pose.position.y = self.y
		odom.pose.pose.position.z = 0.0
		odom.pose.pose.orientation.x = q[0]
		odom.pose.pose.orientation.y = q[1]
		odom.pose.pose.orientation.z = q[2]
		odom.pose.pose.orientation.w = q[3]
		odom.twist.twist.linear.x = ds / (delta_time.nanoseconds/1000000000)
		odom.twist.twist.angular.z = dth / (delta_time.nanoseconds/1000000000)
  
		self.odom_pub.publish(odom) # publish message
	
	def cmd_callback(self, cmdvel):
		self.cmd_vel = cmdvel
  
	def handle_cmd_vel(self):
		wheel_track=self.get_parameter('wheel_track').get_parameter_value()._double_value
		self.max_x_speed = self.get_parameter('max_x_speed').get_parameter_value()._double_value
		self.max_z_speed = self.get_parameter('max_z_speed').get_parameter_value()._double_value
  
		if self.cmd_vel: 
			req_theta = self.cmd_vel.angular.z * (wheel_track / 2.0)

			# limits requested z velocity to max z velocity
			if self.max_z_speed: # if max_z_speed is not 0
				theta = min(abs(req_theta), self.max_z_speed)

				if req_theta < 0:
					theta *= -1
			
			dist_left = self.cmd_vel.linear.x - theta
			dist_right = self.cmd_vel.linear.x + theta
			req_velocity = abs(max(dist_left, dist_right))
			drive_vel = min(req_velocity, 0.3)  # .3 m/s is the max allowed by neato api

			# limits requested x velocity to max x velocity
			if self.max_x_speed: # if max_x_speed is not 0
				drive_vel = min(drive_vel,self.max_x_speed)
   
			if drive_vel == 0:
				if self.moving_prev:
					SetMotorWheels(
						1, 1, 1
					)  # 0,0,0 does not stop Neato. Issue 1,1,1 to go forward 1mm and stop.
			else:
				SetMotorWheels(
					int(dist_left * 1000), int(dist_right * 1000), int(drive_vel * 1000)
				)

			self.moving_prev = drive_vel > 0
	
	def laserPub(self):
		# set header
		self.scan.header.stamp = self.get_clock().now().to_msg()
		self.scan.header.frame_id = self.get_parameter('laser_frame').get_parameter_value().string_value
  
		self.scan.ranges = []
		self.scan.intensities = []
  
		scan_reading = GetLDSScan()
  
		for i in range(360):
			if scan_reading[i][2] == 0:
				self.scan.ranges.append(scan_reading[i][0] / 1000.0)
				self.scan.intensities.append(scan_reading[i][1])
			else:  # error condition, ignore
				self.scan.ranges.append(0)
				self.scan.intensities.append(0)
			 
		self.laser_pub.publish(self.scan)  	
  
def quaternion_from_euler(ai, aj, ak):
	ai /= 2.0
	aj /= 2.0
	ak /= 2.0
	ci = math.cos(ai)
	si = math.sin(ai)
	cj = math.cos(aj)
	sj = math.sin(aj)
	ck = math.cos(ak)
	sk = math.sin(ak)
	cc = ci*ck
	cs = ci*sk
	sc = si*ck
	ss = si*sk

	q = np.empty((4, ))
	q[0] = cj*sc - sj*cs
	q[1] = cj*ss + sj*cc
	q[2] = cj*cs - sj*sc
	q[3] = cj*cc + sj*ss

	return q
		
def main(args=None):
	rclpy.init(args=args)

	node = neato_node()
 
	while rclpy.ok():
		try:
			rclpy.spin_once(node, timeout_sec=0.3)
			if node.useOdom:
				node.odomPub()
			if node.useLaser:
				node.laserPub()
			node.handle_cmd_vel()
		except KeyboardInterrupt:
	 		break

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	
	node.destroy_node()
	rclpy.shutdown()
	SetLED(BacklightStatus.Off, ButtonColors.Off)
	SetLDSRotation(False)
	TestMode(False)
	PlaySound(Sounds.UserTerminatedCleaning)
	


if __name__ == '__main__':
	main()
