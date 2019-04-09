#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix		#Simulation and gps fix
from nav_msgs.msg import Odometry
import pdb					#Debugger
import tf
import numpy as np

#### TO CHANGE ROS WORKING DIRECTORY
#### export ROS_HOME=$HOME 

#### USAGE
#    USE ESTIMATOR TO GET VEHICLE ODOM FIRST

class SteeringMethods:
    def __init__(self,filename='default'):
	#### TODO: RAISE ERROR IF filename is default
        #### Callback Flags, length = number of callbacks 
        self.flag = 0
	#### Get waypoints
	self.pathArray = self.readText(filename)

        #### ROS TOPICS
        #--------------
        # GET POSITION TOPIC
	self.subPos = rospy.Subscriber('/vehicle/odom',Odometry,self.odom_cb)

	#### Define Some Variables
	self.LA = lookAhead
	self.WB = wheelBase
	self.SR = steeringRatio
	pdb.set_trace()

        #### PRINT MESSAGES FOR DEBUG
        #print "\n speed cmd: {} \n speed: {} \n throttle: {} \n brake: {} \n totalError: {}".format(self.vx_desired,self.vx_measure,throttleMsg.pedal_cmd,brakeMsg.pedal_cmd,PIDcontroller.errorTotalReturn())

    #### CALLBACK FUNCTIONS
    def odom_cb(self,msg):
	self.pose.x = msg.pose.pose.position.x
	self.pose.y = msg.pose.pose.position.y
	#self.pose.x = msg.pose.pose.position.z
	q = msg.pose.pose.orientation
	euler = tf.transformations.euler_from_quaternion(q)
	self.roll = euler[0]
	self.pitch = euler[1]
	self.yaw = euler[2]
	self.linearX = msg.twist.twist.linear.x
	self.angularZ = msg.twist.twist.angular.z	
        self.flag = 1					#Set flag

    #-----------------------------

    #### MISC FUNCTIONS
    def satValues(self,value,satLower, satUpper):
        if value >= satUpper:
            return satUpper
        elif value <= satLower:
            return satLower
        else:
            return value

    def readText(self,filename = 'rtk_waypoints.dat'):
	txt =np.loadtxt(filename,delimiter=',')
	return txt

    def angleDiff(value,angle):
	#### SEE IMPLEMENTATION: https://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
	return np.fmod((angle+np.pi),2*np.pi)-np.pi

    #------------------------------
    #### Path Following Algorithms   
    def methodPurePursuit(self,LA=10.0,wheelBase = 2.85,steeringRatio = 14.8):
	if self.flag==1:
		#### Find closest waypoint 
		diff = self.pathArray - np.array[self.pose.x,self.pose.y]
		diffSq = diff[:,0]**2 + diff[:,1]**2
		minInd = np.argmin(diffSq)
		distList = np.sqrt(diffSq)
		#minval = np.min(np.sqrt(diffsq))

		i = mindInd
		#### Find waypoint some lookahead away
		while i >= 0:
			if distList[i] >= lookAhead:
				targetPoint = i  
				break
			else:
				i += 1
		       		if i > len(self.pathArray):
					i = 0
		#### Compute relative bearing
		targetX = self.pathArray[self.targetPoint][0]
		targetY = self.pathArray[self.targetPoint][1]
		#### See terminology: https://aviation.stackexchange.com/questions/8000/what-are-the-differences-between-bearing-vs-course-vs-direction-vs-heading-vs-tr#8947
		eastingBearing = np.atan2(targetY - self.pose.y, targetX - self.pose.x)
		relativeBearing = self.angleDiff(eastingBearing,self.yaw)			#Heading Error basically
	
		#### Compute desired curvature
		curv = 2*np.sin(relativeBearing)/distList[targetPoint]
	
		#### Compute steering command (ackermann geometry)
		steercmd = SteeringRatio*np.atan2(WheelBase*curv,1)
	
		return steercmd, curv, eastingBearing,relativeBearing, targetPoint

	   
if __name__=="__main__":
    rospy.init_node("long_controller_node")
    try:
        steeringmethods = SteeringMethods()
    except rospy.ROSInterruptException:
        pass

    


