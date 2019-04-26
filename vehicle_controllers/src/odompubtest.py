#!/usr/bin/env python

#### WIP SCRIPT #### 
#### THIS SCRIPT IS USED TO CREATE FUSE SENSOR IN VEHICLE ####
#### This requires xsens_mti_ros_node custom_msgs package ####
#### See their old github #### 
#### Create a kalman filter class #### 
import rospy
from custom_msgs.msg import positionEstimate
from custom_msgs.msg import orientationEstimate
from nav_msgs.msg import Odometry
from vehicle_controllers import customOdom2
import tf
import utm


#### global variables
flag = [0,0]
x=0
y=0
z=0
roll=0
pitch=0
yaw=0
global x, y, z, roll, pitch, yaw

rospy.init_node("sensor_fusion_node")

def ori_cb(msg):
	global roll, pitch, yaw, flag
	roll = msg.roll
	pitch = msg.pitch
	yaw = msg.yaw
	flag[0] = 1

def pos_cb
	global x, y, z flag
	temp = utm.from_latlon(msg.latitude, msg.longitude)
	x = temp[0]
	y = temp[1]
	flag[1] = 1
	
rospy.Subscriber("/mti/filter/orientation", orientationEstimate, ori_cb)
rospy.Subscriber("/mti/filter/position", positionEstimate, pos_cb)
oEulerPub = rospy.Publisher("/vehicle/odom2", customOdom2, queue_size=1)
oQuatPub = rospy.Publisher("/vehicle/odom", Odometry, queue_size=1)

odomQuat = Odometry()
odomEuler = customOdom2()

count = 1
rate = rospy.Rate(50)
while not rospy.is_shutdown():
		
	#### Fill in the messages for quaternion odometry 
	quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
	odomQuat.pose.pose.orientation.x = quat[0]
	odomQuat.pose.pose.orientation.y = quat[1]
	odomQuat.pose.pose.orientation.z = quat[2]
	odomQuat.pose.pose.orientation.w = quat[3]
	odomQuat.pose.pose.position.x = x	
	odomQuat.pose.pose.position.y = y	
	odomQuat.pose.pose.position.z = z	
	odomQuat.frame_id = "Kenny"
	odomQuat.header.seq = count
	odomQuat.stamp = rospy.Time.now()
	#### Fill in euler odometry
	odomEuler.x = x
	odomEuler.y = y
	odomEuler.z = z
	odomEuler.roll = roll
	odomEuler.pitch = pitch
	odomEuler.yaw = yaw
	odomEuler.header.seq = count
	odomEuler.stamp = rospy.Time.now()	
	#### Publish 	
	oQuatPub.publish(odomQuat)
	oEulerPub.publish(odomEuler)
	#### Sleep and count
	count += 1
	rate.sleep()





