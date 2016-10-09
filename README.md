Autonomous Two-Wheeled Rover
==============================
This project contains code for a small autonomous rover capable of indoor navigation. 

The rover is a work in progress. It is currently capable of avoid head-on collisions using readings from an ultrasonic distance sensor. 

The inspiration for this project comes from [Prasantha Jayakody's Rover project](https://www.hackster.io/peejster/rover-c42139?ref=search&ref_id=rover&offset=0). 

### Software Installation
* [Raspbian](http://www.raspbian.org/)
* [Robot Operating System - Indigo](http://www.ros.org/)
* Python 2.7
* `rospy` [http://wiki.ros.org/rospy](http://wiki.ros.org/rospy)
* `RPi.GPIO` [https://pypi.python.org/pypi/RPi.GPIO](https://pypi.python.org/pypi/RPi.GPIO)

1. [Install ROS Indigo on the Raspberry Pi](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Indigo%20on%20Raspberry%20Pi)
2. Clone this repo into `~/ros_catkin_ws/src/`
3. Build the catkin workspace again:
  ```
    sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/indigo -j2
  ```
  
4. Test installation by launching the rover `roslaunch rover rover.launch`

### Useful rostopic commands for testing
Toggle between manual and auto control modes  
`rostopic pub /rvr_mode std_msgs/String \'{\"mode\":\"auto\"}\'`

Publish a measurement to the ultrasonic sensor  
`rostopic pub /rvr_ultrasonic std_msgs/String \'{\"distance\":7.92}\'`

Publish a movement to the motors  
`rostopic pub /rvr_motors std_msgs/String \'{\"direction\":\"forward\"\,\"duration\":0.2}\'`
