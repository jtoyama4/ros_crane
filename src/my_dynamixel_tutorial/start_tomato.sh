#!/bin/sh

rosrun my_dynamixel_tutorial tilt_starter.py
roslaunch my_dynamixel_tutorial start_10_tilt_controller.launch
roslaunch my_dynamixel_tutorial start_10_pan_controller.launch
rosrun my_dynamixel_tutorial tilt_starter.py
rostopic pub -1 /tilt2_controller/command std_msgs/Float64 -- 3.2
#rosrun my_dynamixel_tutorial pan_starter.py 
rosrun my_dynamixel_tutorial tilt_all_fixer.py

