import rospy
from std_msgs.msg import Float64

rospy.init_node('test')
pub = rospy.Publisher('/pan9_controller/command', Float64, queue_size = 10)
rate = rospy.Rate(1)
speed = 1.0
for i in range(10):
    pub.publish(speed)
    rate.sleep()
    speed = -speed
