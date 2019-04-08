#!/usr/bin/env python
import rospy
import dbw_mkz_msgs.msg import BrakeCmd
import dbw_mkz_msgs.msg import ThrottleCmd
from std_msgs.msg import Float64
from pid import PID

#### TO CHANGE ROS WORKING DIRECTORY
#### export ROS_HOME=$HOME 

#### USAGE
#    SIMPLY IMPORT AND CREATE AN INSTACE OF CLASS LongController
#    or run it 

class LongController:
    def __init__(self):
        #### ROS TOPICS
        #--------------
        # TWIST CONTAINS FORWARD/ANGULAR VELOCITY
        self.subTwist = rospy.Subscriber('/vehicle/twist',TwistStamped,self.twist_cb)
        # RECEIVING COMMANDS TOPIC
        self.subCmd = rospy.Subscriber('long_controller/cmd_vel',Float64,self.cmd_cb)
        # COMMAND THROTTLE TOPIC
        self.pubThrottle = rospy.Publisher('/vehicle/throttle_cmd',ThrottleCmd,queue_size)
        # COMMAND BRAKE TOPIC
        self.pubBrake = rospy.Publisher('/vehicle/brake_cmd',BrakeCmd)

        #### CREATE MESSAGE OBJECTS FOR PUBLISHING
        throttleMsg = ThrottleCmd()
        brakeMsg = BrakeCmd()
        # initialize them
        throttleMsg.enable = True
        throttle.pedal_cmd_type = 2 #1: 0.15 to 0.80  #2: 0-1
        brakeMsg.pedal_cmd_type = 2 #1: 0.15 to 0.50, #2: 0-1

        #### CREATE PID OBJECT
        PIDcontroller=PID()


        #### PUBLISH LOOP
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            #### Enter loop only when callbacks happen
            if sum(self.flag) == len(self.flag):
                vxError = self.vx_desired - self.vx_measure 
                PIDcontroller.update(


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
        elif value <= satLow:
            return satLower
        else:
            return value


if __name__="__main__":
    rospy.init_node("long_controller_node")
    try:
        longcontroller = LongController()
    except rospy.ROSInterruptException:
        pass

    


