#!/usr/bin/env python
import rospy
from dbw_mkz_msgs.msg import SteeringCmd
from pid import PID
from steering_methods import SteeringMethods
import pdb

#### TO CHANGE ROS WORKING DIRECTORY
#### export ROS_HOME=$HOME 
#### CHANGE IN package.xml file

#### USAGE
#    SIMPLY IMPORT OR CREATE AN INSTACE OF CLASS LongController
#    or run it 

class LatController:
    def __init__(self):
        #### Callback Flags, length = number of callbacks 
        self.pubSteer = rospy.Publisher('/vehicle/steering_cmd',SteeringCmd,queue_size=1)		# TOPICS
        SteeringMsg = SteeringCmd()	    # CREATE MESSAGE OBJECTS FOR PUBLISHING
        SteeringMsg.enable = True	    # initialize them
	
	#### CREATE SteeringFeedForward Object. Pass in the waypoints
	steeringFF = SteeringMethods('circlefixed_example.dat')	#Using default mkz values, otherwise specify them: LookAhead = 10.0, WheelBase = 2.85, SteeringRatio = 14.8

	#### CREATE LOOP 
	rate=rospy.Rate(50)
	while not rospy.is_shutdown():
		# methodPurePursuit function 
		# will use default values, otherwise specify them: lookAhead=self.LA,WheelBase=self.WB,SteeringRatio=self.SR
		# Returns steercmd, curv, eastingBearing,relativeBearing, self.targetPoint
		steercmd, curv, absoluteBearing, relativeBearing, targetPoint = SteeringFF.methodPurePursuit()
		SteeringMsg.steering_wheel_angle_cmd = steercmd		
		pubSteer.publish(SteeringMsg)
		rate.sleep()

    
if __name__=="__main__":
    rospy.init_node("lat_controller_node")
    try:
        latcontroller = LatController()
    except rospy.ROSInterruptException:
        pass

    


