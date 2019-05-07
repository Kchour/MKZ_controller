# Lincoln MKZ simulator
Adapted from dbw_mkz_gazebo (dataspeed inc.) to fit our needs. Contains some helper classes for controlling the vehicle. The package can be used on the real mkz or polaris


# RosCallbackDefine Class
Contains a python helper class called RosCallbackDefine to publish/subscribe to topics more easily
## API
* RosCallbackDefine(vehicle)
	* vehicle: "MKZ", "POLARIS"
* return_states()
	* returns an array of length 4 if internal callbacks are happening
	* returns an array of length 1 otherwise
Returns the states of the vehicle (xdot,x,y, yaw)
* publish_vehicle_long(throttle, brake)
	* throttle: float between 0-1
	* brake: float between 0-1
* publish_vehicle_lat(steering)
	* steering: float between 0-1
## Usage
```python
import rospy
from vehicle_controllers.utility import RosCallbackDefine
topic_helper = RosCallbackDefine("MKZ")
rate = rospy.Rate(50)
while not rospy.is_shutdown():
	states = topic_helper.return_states()
	if len(states) > 1:
		""" Algorithm here """
		""" returns throttleCmd, brakeCmd, steerCmd """
		topic_helper.publish_vehicle_long(throttleCmd,brakeCmd)
		topic_helper.publish_vehicle_lat(steerCmd)
	rate.sleep()
```

PID and steering classes are also avaible via (check lat or long_control in /node folder for examples):
```python
from vehicle_controllers.algorithm import PID
from vehicle_controllers.algorithm import SteeringMethods

```

## Dependencies for callback helper
The following messages are required (either build from source or install them)
1. dbw_mkz_msgs
	1. Go to Dataspeed inc. bitbucket (dbw_mkz_ros)
1. pacmod_msgs
	1. Get astuff_msgs from their github


# To Run the Simulator
1. Install ROS Kinetic with Ubuntu 16.04
1. Install dbw_mkz_simulator from dataspeed inc.: https://bitbucket.org/DataspeedInc/dbw_mkz_simulation (use the deprecated version)
# To build and run
1. place infrasim2 into your ~/yourcatkin_ws/src
1. catkin_make or catkin build
1. source ~/yourcatkin_ws/devel/setup.bash
1. roslaunch infrasim2 controller_test_infrasim.launch
# Additional parameters
## To specify the initial location (GPS coordinates)
1. Go to /urdf/mkz_test.urdf.xacro and change lat/lon there
## To change initial orientation/location (radians/meters)
1. Go to /yaml/controller_single_vehicle_test_track_infrasim.yaml
1. Change the yaw/xyz value here 

# Running the Controllers
1. Provide combined odom topic via
```
rosrun vehicle_controllers odompubtest [OPTIONS]

Options:
	MKZSIM		Running on a simulation	
	MKZREAL		Running on the actual mkz vehicle
	POLARIS		Running on the actual polaris vehicle
```

1. Launch the simulation by following the steps above
1. Generate a list of waypoints (delimited by ','...see /nodes folder) or collect waypoints using the waypoint_collect script under /nodes folder.
	1. An example set of waypoints are provided by the file "circlefixed_example.dat". It is a circle with radius 20 meters. The vehicle in the stimulation has already been set up to follow it (with the correct orientation and positioned near the start of the waypoints).
	1. Else,use a teleop package to move the vehicle: https://wiki.ros.org/teleop_twist_keyboard
```
rosrun vehicle_controllers collect_waypoints
```
1. Launch the controllers via
``` 
roslaunch vehicle_controllers controllaunch.launch
```
1. Feel free to modify the parameters in the launch file. HINT: You can also run the waypoint_collect script from the launch file

# Topics
## Relevant Topics Published
* /vehicle/twist
* /vehicle/throttle_report
* /vehicle/steering_report
* /vehicle/brake_report

## Relevant Topics Subscribed
* /vehicle/cmd_vel
* /vehicle/throttle_cmd
* /vehicle/steering_cmd
* /vehicle/brake_cmd
1. For more info, see dbw_mkz_ros dataspeed inc. repository: https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src README.pdf
1. For message api, see http://docs.ros.org/melodic/api/dbw_mkz_msgs/html/index-msg.html
1. Note: use either /vehicle/cmd_vel OR the 3 actuators directly...not both at the same time!


# Troubleshooting
Dont forget to source your setup.bash from ~/catkin_ws/devel/
```
source ~/catkin_ws/devel/setup.bash

```
# TODO
1. Add more states returned to odom1,2 topics
1. Implement EKF/UKF (actual sensor fusion)
1. Waypoint planner and array topic that grows
1. Ros moveit?

