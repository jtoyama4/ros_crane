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
pub7 = rospy.Publisher('/tilt7_controller/command', Float64, queue_size = 10)
pub8 = rospy.Publisher('/tilt8_controller/command', Float64, queue_size = 10)
pub9 = rospy.Publisher('/tilt9_controller/command', Float64, queue_size = 10)
pub10 = rospy.Publisher('/tilt10_controller/command', Float64, queue_size = 10)

pubs = [pub2, pub3, pub5, pub6, pub7, pub8, pub9, pub10]
fixed_angles = [3.2, 1.8, 0.8, 0.5, 2.2, -0.6, 2.4, 0.8]
rate = rospy.Rate(1.0)

count = 2
for pub, angle in zip(pubs, fixed_angles):
    pub.publish(angle)
    if count==4:
        count = 5
    print count
    count += 1
    rate.sleep()

print "tilt fixed"
