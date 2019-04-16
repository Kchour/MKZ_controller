#!/usr/bin/env python

import rospy 

class RosCallbackDefine:
	def __init__(self,vehicle):
		if vehicle = "MKZ":
			self.init_mkz()
			self.flag = [0,0,0]	#3 callback functions for now	
		elif vehicle = "POLARIS":
			self.init_polaris()
	#### MKZ ####
	def init_mkz(self):
		from dbw_mkz_msgs.msg import SteeringCmd
		from dbw_mkz_msgs.msg import ThrottleCmd
		from dbw_mkz_msgs.msg import BrakeCmd
		from std_msgs.msg import Float64
		from geometry_msgs.msg import TwistStamped
		#### LONGITUDINAL TOPICS #### 
		# TWIST CONTAINS FORWARD/ANGULAR VELOCITY
		self.subTwist = rospy.Subscriber('/vehicle/twist',TwistStamped,self.twist_cb)
		# RECEIVING COMMANDS TOPIC
		self.subCmd = rospy.Subscriber('long_controller/cmd_vel',Float64,self.cmd_cb)
		# COMMAND THROTTLE TOPIC
		self.pubThrottle = rospy.Publisher('/vehicle/throttle_cmd',ThrottleCmd,queue_size =1)
		# COMMAND BRAKE TOPIC
		self.pubBrake = rospy.Publisher('/vehicle/brake_cmd',BrakeCmd,queue_size=1)
		
		#### LATERAL TOPICS #### 
		self.pubSteer = rospy.Publisher('/vehicle/steering_cmd',SteeringCmd,queue_size=1)		# TOPICS
		self.subOdom = rospy.Subscriber('/vehicle/odom',Odometry,self.odom_cb)
		
		#### CREATE PUBLISHING MESSAGES 
		self.throttleMsg = ThrottleCmd()
		self.brakeMsg = BrakeCmd()
		self.steeringMsg = SteeringCmd()
		
		self.throttleMsg.enable = True
		self.throttleMsg.pedal_cmd_type = 2
		self.brakeMsg.enable = True
		self.brakeMsg.pedal_cmd_type = 2
		self.steeringMsg.enable = True

	def return_mkz(self):
		if sum(self.flag) == length(self.flag):
			return [[self.vx_measure, self.vx_desired, self.pose_x, self.pose_y, self.yaw],
				[self.pubThrottle, self.pubBrake, self.pubSteer]]
		else:
			return 0
	def publish_mkz(self,throttle,brake,steering):
		# Assign the values passed
		self.throttleMsg.pedal_cmd = throttle
		self.brakeMsg.pedal_cmd = brake
		self.steeringMsg.steering_wheel_angle_cmd = steering
		# Publish the messages	
		self.pubThrottle.Publish(self.throttleMsg)
		self.pubBrake.Publish(self.brakeMsg)
		self.pubSteer.Publish(self.steeringMsg)
	# TODO RID OF TWIST_CB #
	def twist_cb(self):
        	self.vx_measure = msg.twist.linear.x
		self.flag[0] = 1 
	def cmd_cb(self,msg):
		self.vx_desired = msg.data
		self.flag[1] = 1
	def odom_cb(self,msg):
		self.pose_x = msg.pose.pose.position.x
		self.pose_y = msg.pose.pose.position.y
		#self.pose.x = msg.pose.pose.position.z
		quat = msg.pose.pose.orientation
		euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
		self.roll = euler[0]
		self.pitch = euler[1]
		self.yaw = euler[2]
		self.linearX = msg.twist.twist.linear.x
		self.angularZ = msg.twist.twist.angular.z	
		self.flag[2] = 1					#Set flag
	#### POLARIS ####
