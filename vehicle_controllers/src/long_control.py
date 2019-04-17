#!/usr/bin/env python
import rospy
from dbw_mkz_msgs.msg import BrakeCmd
from dbw_mkz_msgs.msg import ThrottleCmd
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped
from pid import PID
from ros_callback import RosCallbackDefine
import pdb

#### TO CHANGE ROS WORKING DIRECTORY
#### export ROS_HOME=$HOME 
####

#### USAGE
#    SIMPLY IMPORT AND CREATE AN INSTANCE OF CLASS LongController
#    or run it 

class LongController:
    def __init__(self):
	#### INITIALIZE DESIRED VEHICLE SPEED
	self.vx_desired = rospy.get_param('desired_speed',5.0)
	
	#### COMMAND VELOCITY TOPIC
        self.subCmd = rospy.Subscriber('long_controller/cmd_vel',Float64,self.cmd_cb)

        #### CREATE PID OBJECT
        PIDcontroller=PID()
        PIDcontroller.setLims(0.0,125.0)       #### ADJUST THE UPPER LIMIT BASED ON EXPERIMENTATION
	
	#### CREATE TOPIC HELPER CLASS
	topic_helper = RosCallbackDefine("MKZ") 

        #### PUBLISH LOOP and Additional parameters
        rate = rospy.Rate(50)
        errTol = 1.0
        while not rospy.is_shutdown():
  	    	#### RETURN STATES
	    	states = topic_helper.return_states()
            	if len(states) > 1:
			linearX = states[0]
			pose_x = states[1]
			pose_y = states[2]
			yaw = states[3]
			publishTrue = True
		else:
			publishTrue = False

		if publishTrue ==True:
			#### Enter loop only when callbacks happen
			vxError = self.vx_desired - linearX 
			PIDcontroller.update(vxError)

			#### Gain scheduling for different regions of error
			if vxError >= 0:
			    if abs(vxError) >= errTol:
				PIDcontroller.setGains(0.005,0.005,0.005)
			    else:
				PIDcontroller.setGains(0.,0.5*vxError**2,0.)
			    
			    u = PIDcontroller.computeControl()      # Compute Control input
			    u = self.satValues(u,0.0,1.0)           # Bound control input
			    throttle_cmd = u               # set throttle
			    brake_cmd = 0.0                # set brake to 0
			elif vxError < 0:
			    if abs(vxError) >= errTol:
				PIDcontroller.setGains(0.05,0.,0.)
			    else:
				PIDcontroller.setGains(0.05,0.,0.)
			     
			    u = PIDcontroller.computeControl()      # Compute Control input
			    u = self.satValues(abs(u),0.0,1.0)           # Bound control input
			    brake_cmd = u                  # set brake
			    throttle_cmd = 0.0             # set throttle to 0

			#### PUBLISH THE MESSAGES
			topic_helper.publish_vehicle_long(throttle_cmd,brake_cmd)
			#### PRINT MESSAGES FOR DEBUG
			print "\n speed cmd: {} \n speed: {} \n throttle: {} \n brake: {} \n totalError: {}".format(self.vx_desired,linearX,throttle_cmd,brake_cmd,PIDcontroller.errorTotalReturn())
                rate.sleep()


    def cmd_cb(self,msg):
        self.vx_desired = msg.data
    #-----------------------------

    #### MISC FUNCTIONS
    def satValues(self,value,satLower, satUpper):
        if value >= satUpper:
            return satUpper
        elif value <= satLower:
            return satLower
        else:
            return value
    
if __name__=="__main__":
    rospy.init_node("long_controller_node")
    try:
        longcontroller = LongController()
    except rospy.ROSInterruptException:
        pass

    


