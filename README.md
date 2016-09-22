Autonomous Two-Wheeled Rover
==============================
This project contains code for a small autonomous rover capable of indoor navigation. 

The rover is a work in progress. It is currently capable of avoid head-on collisions using readings from an ultrasonic distance sensor. 

The inspiration for this project comes from [Prasantha Jayakody's Rover project](https://www.hackster.io/peejster/rover-c42139?ref=search&ref_id=rover&offset=0). 

### Software required
* [Raspbian](http://www.raspbian.org/)
* [Robot Operating System](http://www.ros.org/)
* Python 2.7
* `rospy` [http://wiki.ros.org/rospy](http://wiki.ros.org/rospy)
* `RPi.GPIO` [https://pypi.python.org/pypi/RPi.GPIO](https://pypi.python.org/pypi/RPi.GPIO)

### Useful rostopic commands for testing
Toggle between manual and auto control modes  
`rostopic pub /rvr_mode std_msgs/String \'{\"mode\":\"auto\"}\'`

Publish a measurement to the ultrasonic sensor  
`rostopic pub /rvr_ultrasonic std_msgs/String \'{\"distance\":7.92}\'`

Publish a movement to the motors  
`rostopic pub /rvr_motors std_msgs/String \'{\"direction\":\"forward\"\,\"duration\":0.2}\'`
