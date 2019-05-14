#!/usr/bin/env python

import rospy 
import tf
import pdb
import numpy as np

class RosCallbackDefine:
	def __init__(self,vehicle):
		self.vehicle = vehicle
		self.vehList = ["MKZ", "POLARIS"]
		self.flag = [0,0]	#2 callback functions for now
		if vehicle == self.vehList[0]:
			self.__init_mkz()
		elif vehicle == self.vehList[1]:
			self.__init_polaris()

	#### PLATFORM AGNOSTIC FUNCTIONS:w
	def return_states(self):
		if sum(self.flag) == len(self.flag):
			return [self.linearX, self.pose_x, self.pose_y, self.yaw]
		else:
			return [0]

	def publish_vehicle_lat(self,steering):
		if self.vehicle == self.vehList[0]:			#### MKZ CASE ####
			# Assign the values passed
			self.steeringMsg.steering_wheel_angle_cmd = steering
			# Publish the messages	
		elif self.vehicle == self.vehList[1]:			#### POLARIS CASE ####			
			self.steeringMsg.command = steering
			self.steeringMsg.rotation_rate = 30.0	# 2 is slow	
		
		#### PUBLISH MESSAGES		
		self.pubSteer.publish(self.steeringMsg)	
		#self.pubEnable.publish(self.enableMsg)
	def publish_vehicle_long(self,throttle,brake):
		if self.vehicle == self.vehList[0]:
			self.throttleMsg.pedal_cmd = throttle
			self.brakeMsg.pedal_cmd = brake
		elif self.vehicle == self.vehList[1]:
			self.throttleMsg.command = throttle
			self.brakeMsg.command = brake	
		#### PUBLISH MESSAGES		
		self.pubThrottle.publish(self.throttleMsg)
		self.pubBrake.publish(self.brakeMsg)
		
		
		#self.pubEnable.publish(self.enableMsg)
	#### CALLBACK FUNCTIONS REUSED ####
	#### TODO RID OF TWIST_CB ####
	
	def __speed_cb(self,msg,args):
		if args[0:] == self.vehList[0]:		#### MKZ CASE ####
			self.linearX = msg.twist.linear.x
			self.flag[0] = 1
		elif args[0:] == self.vehList[1]:	#### POLARIS CASE ####
			#self.linearX = msg.data		#### /as_tx/vehicle_speed 
			self.linearX=np.sqrt(msg.NedVel.x**2 + msg.NedVel.y**2)
			self.speedMsg.data = self.linearX		
			self.pubSpeed.publish(self.speedMsg) 
			self.flag[0] = 1

	def __odom_cb(self,msg,args):
		self.pose_x = msg.x#msg.pose.pose.position.x
		self.pose_y = msg.y#msg.pose.pose.position.y
		#quat = msg.pose.pose.orientation
		#euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
		self.roll = msg.roll # euler[0]
		self.pitch = msg.pitch # euler[1]
		self.yaw = msg.yaw #euler[2]
		#self.linearX = msg.twist.twist.linear.x
		#self.angularZ = msg.twist.twist.angular.z	
		self.flag[1] = 1

	#### status indicator callback for Polaris
	def __systemFlag_cb(self,msg,args):
		self.enabledFlag = msg.data
		if self.enabledFlag == True:
                	self.throttleMsg.clear_override = False
               		self.throttleMsg.clear_faults = False
                	self.brakeMsg.clear_override = False
                	self.brakeMsg.clear_faults = False
		        self.steeringMsg.clear_override = False
		        self.steeringMsg.clear_faults = False
			self.autonomousModeOnce = True
		elif self.autonomousModeOnce == False:
                	self.throttleMsg.clear_override = True
               		self.throttleMsg.clear_faults = True
                	self.brakeMsg.clear_override = True
                	self.brakeMsg.clear_faults = True
		        self.steeringMsg.clear_override = True
		        self.steeringMsg.clear_faults = True
			self.pubBrake.publish(self.brakeMsg)	
			self.pubThrottle.publish(self.throttleMsg)	
			self.pubSteer.publish(self.steeringMsg)	

	#### MKZ ####
	def __init_mkz(self):
		from dbw_mkz_msgs.msg import SteeringCmd
		from dbw_mkz_msgs.msg import ThrottleCmd
		from dbw_mkz_msgs.msg import BrakeCmd
		from std_msgs.msg import Float64
		from geometry_msgs.msg import TwistStamped
		from nav_msgs.msg import Odometry
		from sensor_msgs.msg import NavSatFix
		from vehicle_controllers.msg import customOdom2


		
		#### CREATE PUBLISHING MESSAGES 
		self.throttleMsg = ThrottleCmd()
		self.brakeMsg = BrakeCmd()
		self.steeringMsg = SteeringCmd()
		
		self.throttleMsg.enable = True
		self.throttleMsg.pedal_cmd_type = 2
		self.brakeMsg.enable = True
		self.brakeMsg.pedal_cmd_type = 2
		self.steeringMsg.enable = True

		# VEHICLE NAME #
		VEH = self.vehList[0]	#  "MKZ"
		#### LONGITUDINAL TOPICS #### 
		# TWIST CONTAINS FORWARD/ANGULAR VELOCITY
		self.subSpeed = rospy.Subscriber('/vehicle/twist',TwistStamped,self.__speed_cb,(VEH))
		# COMMAND THROTTLE TOPIC
		self.pubThrottle = rospy.Publisher('/vehicle/throttle_cmd',ThrottleCmd,queue_size =1)
		# COMMAND BRAKE TOPIC
		self.pubBrake = rospy.Publisher('/vehicle/brake_cmd',BrakeCmd,queue_size=1)
		
		#### LATERAL TOPICS #### 
		self.pubSteer = rospy.Publisher('/vehicle/steering_cmd',SteeringCmd,queue_size=1)		# TOPICS
		self.subOdom = rospy.Subscriber('/vehicle/odom2',customOdom2,self.__odom_cb,(VEH))


	#### POLARIS ####
	def __init_polaris(self):
		from std_msgs.msg import Float64	#for speed
		from pacmod_msgs.msg import SystemCmdFloat
		from pacmod_msgs.msg import SteerSystemCmd
		from vehicle_controllers.msg import customOdom2
		from std_msgs.msg import Bool
		from vn300.msg import ins
		# VEHICLE NAME #
		VEH = self.vehList[1]	# "POLARIS"

		#### State of the vehicle
		self.enabledFlag = False
		self.autonomousModeOnce = False		#only clear faults BEFORE getting into auto mode ONCE
		
                #### CREATE PUBLISHING MESSAGES
		self.throttleMsg = SystemCmdFloat()
		self.brakeMsg = SystemCmdFloat()
		self.steeringMsg = SteerSystemCmd()
		self.speedMsg = Float64()	

		self.throttleMsg.enable = True
		self.brakeMsg.enable = True
		self.steeringMsg.enable = True
		self.enabledFlag = False	#Not in autonomous mode = false flag

		self.throttleMsg.ignore_overrides=False
		self.brakeMsg.ignore_overrides=False
		self.steeringMsg.ignore_overrides=False

		#### LONGITUDINAL INFO ####
		#self.subSpeed = rospy.Subscriber("/as_tx/vehicle_speed",Float64, self.__speed_cb,(VEH))
		self.subSpeed = rospy.Subscriber("/vectornav/ins",ins, self.__speed_cb,(VEH))
		self.pubThrottle = rospy.Publisher("/as_rx/accel_cmd",SystemCmdFloat,queue_size=1)
		self.pubBrake = rospy.Publisher("/as_rx/brake_cmd",SystemCmdFloat,queue_size=1)
		
		#### LATERAL INFO ####
		self.odomSub = rospy.Subscriber("/vehicle/odom2", customOdom2, self.__odom_cb,(VEH))
		self.pubSteer = rospy.Publisher("/as_rx/steer_cmd",SteerSystemCmd,queue_size=1)
		self.statusFlag = rospy.Subscriber("/as_tx/enabled",Bool,self.__systemFlag_cb,(VEH))

		#### ENABLE INFO ####
		self.pubSpeed = rospy.Publisher("/vectornav/velTEST",Float64,queue_size=1)


		
	

		
		
