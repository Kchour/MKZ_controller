#!/usr/bin/env python
import rospy
class PID:
    def __init__(self,kp=1,ki=0.0,kd=0.0,satLower=-100., satUpper=100.):
        self.errorNow = 0.0
        self.errorTot = 0.0
        self.errorPrev = 0.0
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.satLower = satLower
        self.satUpper = satUpper
    
    def update(self,error):
        #### set previous error, current error, and accumulated error
        self.errorPrev = self.errorNow
        self.errorNow = error
        self.errorTot += error

    def errorTotalReturn(self):
        return self.errorTot

    def setGains(self,kp,ki,kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def setLims(self,satLower,satUpper):
        self.satLower = satLower
        self.satUpper = satUpper

    def satValues(self,value,satLower, satUpper):
        if value >= satUpper:
            return satUpper
        elif value <= satLow:
            return satLower
        else:
            return value

    def compute_control(self):
        u = self.kp*self.errorNow + self.ki*self.errorTot + kd * (self.errorNow - self.errorPrev)

        return u


