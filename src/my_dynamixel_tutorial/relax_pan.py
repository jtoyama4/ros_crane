#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
import sys
import numpy as np

rospy.init_node('fixer')
pub7 = rospy.Publisher('/pan7_controller/command', Float64, queue_size = 10)
pub8 = rospy.Publisher('/pan8_controller/command', Float64, queue_size = 10)
pub9 = rospy.Publisher('/pan9_controller/command', Float64, queue_size = 10)
pub10 = rospy.Publisher('/pan10_controller/command', Float64, queue_size = 10)

pubs = [pub7, pub8, pub9, pub10]
fixed_angles = [0.0, 0.0, 0.0, 0.0]
rate = rospy.Rate(1)

for pub, angle in zip(pubs, fixed_angles):
    pub.publish(angle)
    rate.sleep()

print "tilt fixed"
