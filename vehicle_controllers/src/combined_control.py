#!/usr/bin/env python
import rospy
from pid import PID
from steering_methods import SteeringMethods
import pdb
from ros_callback import RosCallbackDefine

#### TO CHANGE ROS WORKING DIRECTORY
#### export ROS_HOME=$HOME 
#### CHANGE IN package.xml file

#### USAGE
#    SIMPLY IMPORT OR CREATE AN INSTACE OF CLASS LongController
#    or run it 
		
class CombController:
    def __init__(self):
		
	#### CREATE SteeringFeedForward Object. Pass in the waypoints
	steeringFF = SteeringMethods('circlefixed_example.dat')	#Using default mkz values, otherwise specify them: LookAhead = 10.0, WheelBase = 2.85, SteeringRatio = 14.8
	topic_helper = RosCallbackDefine("MKZ")		
	#### CREATE LOOP 
	rate=rospy.Rate(50)
	while not rospy.is_shutdown():
		# methodPurePursuit function 
		# will use default values, otherwise specify them: lookAhead=self.LA,WheelBase=self.WB,SteeringRatio=self.SR
		# Returns steercmd, curv, eastingBearing,relativeBearing, self.targetPoint
		states = topic_helper.return_mkz()
		if len(states) > 1:
			linearX = states[0]
			pose_x = states[1]
			pose_y = states[2]
			yaw = states[3]
			print states
			steercmd, curv, absoluteBearing, relativeBearing, targetPoint = steeringFF.methodPurePursuit(pose_x,pose_y,yaw)		# Calculate steering 
			topic_helper.publish_mkz_lat(steercmd)									# Publish the steering cmd	
		rate.sleep()

    
if __name__=="__main__":
    rospy.init_node("Comb_controller_node")
    try:
        CombController()
    except rospy.ROSInterruptException:
        pass

def cmd_cb(self,msg):
	self.vx_desired = msg.data
	self.flag[1] = 1
self.vx_desired = states[1]
