#!/usr/bin/env python

import rospy 
import tf

class RosCallbackDefine:
	def __init__(self,vehicle):
		self.vehicle = vehicle
		self.vehList = ["MKZ", "POLARIS"]
		if vehicle == self.vehList[0]:
			self.__init_mkz()
			self.flag = [0,0]	#3 callback functions for now	
		elif vehicle == self.vehList[1]:
			self.__init_polaris()

	#### PLATFORM AGNOSTIC FUNCTIONS:w
	def return_states(self):
		if self.vehicle == self.vehList[0]:
			return self.__return_mkz()

	def publish_vehicle_lat(self,steering):
		if self.vehicle == self.vehList[0]:
			self.__publish_mkz_lat(steering)	
	
	def publish_vehicle_long(self,throttle,brake):
		if self.vehicle == self.vehList[0]:
			self.__publish_mkz_long(throttle,brake)	
	#### MKZ ####
	def __init_mkz(self):
		from dbw_mkz_msgs.msg import SteeringCmd
		from dbw_mkz_msgs.msg import ThrottleCmd
		from dbw_mkz_msgs.msg import BrakeCmd
		from std_msgs.msg import Float64
		from geometry_msgs.msg import TwistStamped
		from nav_msgs.msg import Odometry
		from sensor_msgs.msg import NavSatFix
		#### LONGITUDINAL TOPICS #### 
		# TWIST CONTAINS FORWARD/ANGULAR VELOCITY
		self.subTwist = rospy.Subscriber('/vehicle/twist',TwistStamped,self.__twist_cb)
		# COMMAND THROTTLE TOPIC
		self.pubThrottle = rospy.Publisher('/vehicle/throttle_cmd',ThrottleCmd,queue_size =1)
		# COMMAND BRAKE TOPIC
		self.pubBrake = rospy.Publisher('/vehicle/brake_cmd',BrakeCmd,queue_size=1)
		
		#### LATERAL TOPICS #### 
		self.pubSteer = rospy.Publisher('/vehicle/steering_cmd',SteeringCmd,queue_size=1)		# TOPICS
		self.subOdom = rospy.Subscriber('/vehicle/odom',Odometry,self.__odom_cb)
		
		#### CREATE PUBLISHING MESSAGES 
		self.throttleMsg = ThrottleCmd()
		self.brakeMsg = BrakeCmd()
		self.steeringMsg = SteeringCmd()
		
		self.throttleMsg.enable = True
		self.throttleMsg.pedal_cmd_type = 2
		self.brakeMsg.enable = True
		self.brakeMsg.pedal_cmd_type = 2
		self.steeringMsg.enable = True

	def __return_mkz(self):
		if sum(self.flag) == len(self.flag):
			return [self.vx_measure, self.pose_x, self.pose_y, self.yaw]
		else:
			return [0]

	def __publish_mkz_lat(self,steering):
		# Assign the values passed
		self.steeringMsg.steering_wheel_angle_cmd = steering
		# Publish the messages	
		self.pubSteer.publish(self.steeringMsg)

	def __publish_mkz_long(self, throttle,brake):
		self.throttleMsg.pedal_cmd = throttle
		self.brakeMsg.pedal_cmd = brake
		
		self.pubThrottle.publish(self.throttleMsg)
		self.pubBrake.publish(self.brakeMsg)
	# TODO RID OF TWIST_CB #
	def __twist_cb(self,msg):
        	self.vx_measure = msg.twist.linear.x
		self.flag[0] = 1 
	def __odom_cb(self,msg):
		self.pose_x = msg.pose.pose.position.x
		self.pose_y = msg.pose.pose.position.y
		quat = msg.pose.pose.orientation
		euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
		self.roll = euler[0]
		self.pitch = euler[1]
		self.yaw = euler[2]
		self.linearX = msg.twist.twist.linear.x
		self.angularZ = msg.twist.twist.angular.z	
		self.flag[1] = 1					#Set flag
	#### POLARIS ####
