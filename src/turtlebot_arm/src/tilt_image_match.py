import cv2
import sys
import numpy as np
import rospy
import joblib

class Video(object):
    def __init__(self, img_dims):
        self.cap0 = cv2.VideoCapture(0)
        self.cap1 = cv2.VideoCapture(1)
        self.img_dims = img_dims

    def get_center(self):
        ret, frame0 = self.cap0.read()
        ret, frame1 = self.cap1.read()
        frame0 = cv2.resize(frame0, self.img_dims)
        frame1 = cv2.resize(frame1, self.img_dims)

        center0 = self.preprocess(frame0) / self.img_dims
        center1 = self.preprocess(frame1) / self.img_dims

        center = np.concatenate([center0, center1])

        return center

    def preprocess(self, frame):
        # Our operations on the frame come here
        frame = cv2.GaussianBlur(frame, (25, 25), 1)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # get only red object
        lower_red = np.array([160, 100, 100])
        upper_red = np.array([180, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)

        res = cv2.bitwise_and(frame, frame, mask=mask)

        gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)

        # get center of object
        ret, thresh = cv2.threshold(gray, 5, 255, cv2.THRESH_BINARY)
        contours = cv2.findContours(thresh, 1, 2)
        cnt = contours[0]
        M = cv2.moments(cnt)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        return np.array([cx, cy])

class Arm(object):
    def __init__(self):
        rospy.init_node('pub_and_sub')
        motor_ids = [2,3,5,6]
        self.pubs = self.init_publisher(motor_ids)
        self.init_subscriber(motor_ids)

    def init_publisher(self, motor_ids):
        pubs = []
        for id in motor_ids:
            exec("self.pub%d =rospy.Publisher('/tilt%d_controller/command', Float64, queue_size=10" % (id, id))
            exec("pubs.append(self.pub%d)" % id)
        return pubs

    def init_subscriber(self, motor_ids):
        for motor in motor_ids:
            exec("rospy.Subscriber('/tilt%d_controller/state', JointState, self.get_pos_%d)" % (motor, motor))

    def get_pos_2(self):
        self.angle2 = msg.current_pos

    def get_pos_3(self):
        self.angle3 = msg.current_pos

    def get_pos_5(self):
        self.angle5 = msg.current_pos

    def get_pos_6(self):
        self.angle6 = msg.current_pos

    def get_angles(self):
        return[self.angle2, self.angle3, self.angle5, self.angle6]

    def publish(self, actions):
        for n, action in enumerate(actions):
            exec("self.pub%d.publish(%f)" % (n, action))

    def get_actions(self):
        angle_2 = self.angle2 + np.random.normal(scale=0.01, size=1)
        angle_3 = self.angle3 + np.random.normal(scale=0.01, size=1)
        angle_5 = self.angle5 + np.random.normal(scale=0.01, size=1)
        angle_6 = self.angle6 + np.random.normal(scale=0.01, size=1)
        return [angle_2, angle_3, angle_5, angle_6]


    def run(self):
        dataset = []
        img_dims = [640, 480]
        video = Video(img_dims)
        while not rospy.is_shuddown():
            actions = self.get_actions()
            self.publish(actions)
            dataset.append([video.get_center(), self.get_angles])

        joblib.dump(dataset, "/Users/jouji/Projects/ros_crane/src/turtlebot_arm/src/data/tilt_image.pkl", compress=3)

