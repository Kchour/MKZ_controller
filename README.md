# To install
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



