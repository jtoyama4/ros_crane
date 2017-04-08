#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
import sys
import numpy as np

rospy.init_node('fixer')
pub2 = rospy.Publisher('/tilt2_controller/command', Float64, queue_size = 10)
pub3 = rospy.Publisher('/tilt3_controller/command', Float64, queue_size = 10)
pub5 = rospy.Publisher('/tilt5_controller/command', Float64, queue_size = 10)
pub6 = rospy.Publisher('/tilt6_controller/command', Float64, queue_size = 10)

pubs = [pub2, pub3, pub5, pub6]
fixed_angles = [3.2, 1.8, 2.5, 0.5]
rate = rospy.Rate(0.5)

for pub, angle in zip(pubs, fixed_angles):
    pub.publish(angle)
    rate.sleep()

print "tilt fixed"
