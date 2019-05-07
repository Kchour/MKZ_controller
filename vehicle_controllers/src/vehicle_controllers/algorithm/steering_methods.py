#!/usr/bin/env python
import numpy as np
import pdb

#### TO CHANGE ROS WORKING DIRECTORY
#### export ROS_HOME=$HOME 

#### USAGE
#    USE ESTIMATOR TO GET VEHICLE ODOM FIRST

class SteeringMethods:
    def __init__(self,filename='default',lookAhead=5.0,wheelBase = 2.87,steeringRatio = 14.8):
	#### TODO: RAISE ERROR IF filename is default
	#### TODO: RAISE ERROR IF INITIAL VEHICLE POSITION IS NOT SET
        #### Callback Flags, length = number of callbacks 
        self.flag = 0
	#### Get waypoints
	self.pathArray = self.readText(filename)
	#### Define Some Variables
	self.LA = lookAhead
	self.WB = wheelBase
	self.SR = steeringRatio

        #### PRINT MESSAGES FOR DEBUG
        #print "\n speed cmd: {} \n speed: {} \n throttle: {} \n brake: {} \n totalError: {}".format(self.vx_desired,self.vx_measure,throttleMsg.pedal_cmd,brakeMsg.pedal_cmd,PIDcontroller.errorTotalReturn())

    #-----------------------------

    #### MISC FUNCTIONS
    def satValues(self,value,satLower, satUpper):
        if value >= satUpper:
            return satUpper
        elif value <= satLower:
            return satLower
        else:
            return value

    def readText(self,filename = 'circlefixed_example.dat'):
	txt  = np.loadtxt(filename,delimiter=',')
	return txt

    def angleDiff(self,a,b):
	#### SEE IMPLEMENTATION: https://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
	diff = a - b
	return np.fmod((diff+np.pi),2*np.pi)-np.pi

    def setParams(self,lookAhead, wheelBase, steeringRatio):
	self.LA = lookAhead
	self.WB = wheelBase
	self.SR = steeringRatio

    def setVehicleState(self, pose_x, pose_y, roll, pitch, yaw, linearX, angularZ):
	self.pose_x = pose_x
	self.pose_y = pose_y
	self.roll = roll
	self.pitch = pitch
	self.yaw = yaw
	self.linearX = linearX
	self.angularZ = angularZ
	self.flag = 1

    #------------------------------
    #### Path Following Algorithms   
    def methodPurePursuit(self,pose_x,pose_y,yaw):
 	wheelBase = self.WB	
	lookAhead = self.LA
	steeringRatio = self.SR
	#### Find closest waypoint 
	diff = self.pathArray - np.array([pose_x,pose_y])
	diffSq = diff[:,0]**2 + diff[:,1]**2
	minInd = np.argmin(diffSq)
	distList = np.sqrt(diffSq)
	#minval = np.min(np.sqrt(diffsq))

	i = minInd
	#i = targetPoint
	#### Find waypoint some lookahead away
	while i >= 0:
		if distList[i] >= lookAhead:
			targetPoint = i  
			break
		else:
			i += 1
			if i > len(self.pathArray)-1:
				i = 0
	#### Compute relative bearing
	targetX = self.pathArray[targetPoint][0]
	targetY = self.pathArray[targetPoint][1]
	#### See terminology: https://aviation.stackexchange.com/questions/8000/what-are-the-differences-between-bearing-vs-course-vs-direction-vs-heading-vs-tr#8947
	absoluteBearing = np.arctan2(targetY - pose_y, targetX - pose_x)
	relativeBearing = self.angleDiff(absoluteBearing,yaw)			#Heading Error basically

	#### Compute desired curvature
	curv = 2*np.sin(relativeBearing)/distList[targetPoint]

	#### Compute steering command (ackermann geometry)
	steercmd = steeringRatio*np.arctan2(wheelBase*curv,1)
	print targetPoint, minInd, distList[i], relativeBearing
	return steercmd, curv, absoluteBearing,relativeBearing, targetPoint

	   

    


