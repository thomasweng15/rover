Autonomous Two-Wheeled Rover
==============================
This project contains code for a small autonomous rover capable of indoor navigation.

The robot's odometry currently performs poorly and limits its ability to navigate. 

## Running the robot
1. On the robot, run `roslaunch rover rover.launch` and `rosrun amcl amcl`
2. On a remote machine, run `roslaunch rover remote.launch`
