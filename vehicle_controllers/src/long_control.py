#!/usr/bin/env python
import rospy
from dbw_mkz_msgs.msg import BrakeCmd
from dbw_mkz_msgs.msg import ThrottleCmd
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped
from pid import PID
import pdb

#### TO CHANGE ROS WORKING DIRECTORY
#### export ROS_HOME=$HOME 
####

#### USAGE
#    SIMPLY IMPORT AND CREATE AN INSTACE OF CLASS LongController
#    or run it 

class LongController:
    def __init__(self):
        #### Callback Flags, length = number of callbacks 
        self.flag = [0,0]

        #### ROS TOPICS
        #--------------
        # TWIST CONTAINS FORWARD/ANGULAR VELOCITY
        self.subTwist = rospy.Subscriber('/vehicle/twist',TwistStamped,self.twist_cb)
        # RECEIVING COMMANDS TOPIC
        self.subCmd = rospy.Subscriber('long_controller/cmd_vel',Float64,self.cmd_cb)
        # COMMAND THROTTLE TOPIC
        self.pubThrottle = rospy.Publisher('/vehicle/throttle_cmd',ThrottleCmd,queue_size =1)
        # COMMAND BRAKE TOPIC
        self.pubBrake = rospy.Publisher('/vehicle/brake_cmd',BrakeCmd,queue_size=1)

        #### CREATE MESSAGE OBJECTS FOR PUBLISHING
        throttleMsg = ThrottleCmd()
        brakeMsg = BrakeCmd()
        # initialize them
        throttleMsg.enable = True
        throttleMsg.pedal_cmd_type = 2 #1: 0.15 to 0.80  #2: 0-1
        brakeMsg.pedal_cmd_type = 2 #1: 0.15 to 0.50, #2: 0-1

        #### CREATE PID OBJECT
        PIDcontroller=PID()
        PIDcontroller.setLims(0.0,125.0)       #### ADJUST THE UPPER LIMIT BASED ON EXPERIMENTATION

        #### PUBLISH LOOP and Additional parameters
        rate = rospy.Rate(50)
        errTol = 1.0
        while not rospy.is_shutdown():
            #### Enter loop only when callbacks happen
            if sum(self.flag) == len(self.flag):
                vxError = self.vx_desired - self.vx_measure 
                PIDcontroller.update(vxError)

                #### Gain scheduling for different regions of error
                if vxError >= 0:
                    if abs(vxError) >= errTol:
                        PIDcontroller.setGains(0.005,0.005,0.005)
                    else:
                        PIDcontroller.setGains(0.,0.5*vxError**2,0.)
                    
                    u = PIDcontroller.computeControl()      # Compute Control input
                    u = self.satValues(u,0.0,1.0)           # Bound control input
                    throttleMsg.pedal_cmd = u               # set throttle
                    brakeMsg.pedal_cmd = 0.0                # set brake to 0
                elif vxError < 0:
                    if abs(vxError) >= errTol:
                        PIDcontroller.setGains(0.05,0.,0.)
                    else:
                        PIDcontroller.setGains(0.05,0.,0.)
                     
                    u = PIDcontroller.computeControl()      # Compute Control input
                    u = self.satValues(abs(u),0.0,1.0)           # Bound control input
                    brakeMsg.pedal_cmd = u                  # set brake
                    throttleMsg.pedal_cmd = 0.0             # set throttle to 0

                #### PUBLISH THE MESSAGES
                self.pubThrottle.publish(throttleMsg)
                self.pubBrake.publish(brakeMsg)

                #### PRINT MESSAGES FOR DEBUG
                print "\n speed cmd: {} \n speed: {} \n throttle: {} \n brake: {} \n totalError: {}".format(self.vx_desired,self.vx_measure,throttleMsg.pedal_cmd,brakeMsg.pedal_cmd,PIDcontroller.errorTotalReturn())
                rate.sleep()


    #### CALLBACK FUNCTIONS
    def twist_cb(self,msg):
        self.vx_measure = msg.twist.linear.x
        self.flag[0] = 1

    def cmd_cb(self,msg):
        self.vx_desired = msg.data
        self.flag[1] = 1
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

    


