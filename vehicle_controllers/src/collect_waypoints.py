# This script collects waypoints and saves them to textfiles
# Specifically, it collects gps coordinates to one file
# and both gps and odometry (meters) into another

#!/usr/bin/env python
import rospy
import math
import numpy as np
import utm
#from geometry_msgs.msg import Point32
#from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from vehicle_controller.msg import customOdom2
#from sensor_msgs.msg import NavSatFix
#from custom_msgs.msg import positionEstimate
#from custom_msgs.msg import orientationEstimate
#from kalman_filter.msg import States
#import scipy.signal
import pdb
import matplotlib.pyplot as plt

class CollectWaypoints():

	def __init__(self):
                self.shutdown_flag = False
		self.flag=0

		self.filename = "odom_waypoints.dat"
		self.pathArray = []
		self.point = [0,0]
		rospy.Subscriber("/vehicle/odom",Odometry,self.odom_callback,("quat"))
		rospy.Subscriber("/vehicle/odom2",customOdom2,self.odom_callback,("euler"))
		##car
		#rospy.Subscriber("/mti/filter/position",positionEstimate,self.mtiCallback) #Need to modify for mti
		##rospy.Subscriber("/odometry/filtered/utm",Odometry,self.posCallback) #Need to modify for mti
		#rospy.Subscriber("/gps/rtkfix",Odometry,self.posCallback)          
		#rospy.Subscriber("/mssp_return5",positionEstimate,self.gpsCallback)  #assume we can use car_filter 
		##kalman filter
		#rospy.Subscriber("/kalman/states",States,self.kalmanCallback)  
		#
		##sim
		#rospy.Subscriber("/vehicle/odom",Odometry,self.posCallback) 
		#rospy.Subscriber("/vehicle/perfect_gps/fix",NavSatFix,self.gpsCallback)
		
                # UNCOMMENT OUT THE FOLLOWING AS NEEDED
		#rospy.Subscriber("/transformed_coordinates"+str(self.value),PointStamped,self.cameraCallback)
		#rospy.Subscriber("/Comm2Contr/position2", positionEstimate, self.gpsCallback)
		#rospy.Subscriber("/gps/rtkfix", Odometry, self.cameraCallback)
		
		# push back via append
  		rate = rospy.Rate(50)
		rospy.on_shutdown(self.shutdown_hook)
		while not rospy.is_shutdown():
			if sum(self.flag) >= 2:
				self.pathArray.append(self.point)
			rate.sleep()
	
	
        def shutdown_hook(self):
            self.shutdown_flag = True
	    self.pathArray = np.asarray(self.pathArray)
   	    np.savetxt(self.filename,self.pathArray.T,delimiter=',') 
	    
	    plt.figure()
	    plt.scatter(self.pathArray[:,0],self.pathArray[:,1])
	    plt.title("rtk")
            plt.show()

	def odom_callback(self,data,args):
            if self.shutdown_flag == False:
		if args == "quat":
			lat_ = data.pose.pose.position.x
			long_ = data.pose.pose.position.y
		else
			lat_ = data.x
			long_ = data.y
		temp=utm.from_latlon(lat_,long_)	
		self.point[0] = temp[0]
		self.point[1] = temp[1]
		self.flag = 1
		print self.point[0],self.point[1] "\n"
                
			
if __name__=='__main__':
	rospy.init_node('collect_waypoints')
	collect_waypoints = CollectWaypoints()
	rospy.spin()
	
		
