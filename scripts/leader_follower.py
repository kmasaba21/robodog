#!/usr/bin/python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
from os import path
from copy import copy
import pickle

# global variables
INF = 20
PI = math.pi
SAFETY_THRESHOLD = 1.0
<<<<<<< HEAD
STOP_THRESHOLD=0.5
=======
STOP_THRESHOLD = .5
# Robot States
>>>>>>> d6873c23713a7bcb1ed5a0f4ade4fbb8d35d6259
MOVE = 1
STOP = 0
# ANGLE_MIN = -7 * PI / 12.0
# ANGLE_MAX = 7 * PI / 12.0
ANGLE_MIN = -1*PI
ANGLE_MAX = PI
FIELD_OF_VIEW = PI/9
KP = 5
KD = 2
RATE = 10

# LeaderFollower class to follow person based off of their legs
class LeaderFollower:
    def __init__(self):
        # Define publisher and Subscriber
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        # self.scan_sub = rospy.Subscriber("base_scan", LaserScan, self.scan_callback, queue_size=1)
        self.scan_sub = rospy.Subscriber("scan", LaserScan, self.scan_callback, queue_size=1)
        # define necessary variables for leader follower
        self.closest_range = INF
        self.rate = rospy.Rate(RATE)
        self.state = 0
        # self.prev_range = 0
        self.ang_vel_error = 0
        self.lin_vel_error = 0.1
        self.prev_lin_vel = 0.1
        # define data collection file name
        self.data_file_name = "motion_data_{}.pickle".format(rospy.Time.now().secs)

    # callback function that updates ranges so that we are only looking at front ranges
    def scan_callback(self, msg):
        # attach time to data
        time_s = rospy.Time.now().secs
        data = {'time': time_s}

        # get ranges from subscriber
        ranges = msg.ranges
        resolution = msg.angle_increment
        min_ang = msg.angle_min
        max_ang= msg.angle_max

        # create dictionary of all ranges in the fron
        front_angles = {}
        m_angle=80
        m_range=60
        for i in range(len(ranges)):
            angle = min_ang + i * resolution

            if m_range > ranges[i]:
               m_angle=angle
               m_range=ranges[i]
<<<<<<< HEAD
            if PI/18-PI >= angle or angle >= PI-PI/18:
=======

            if FIELD_OF_VIEW+ANGLE_MIN >= angle or angle >= ANGLE_MAX-FIELD_OF_VIEW:
>>>>>>> d6873c23713a7bcb1ed5a0f4ade4fbb8d35d6259
                front_angles[angle] = ranges[i]

        #rospy.logerr("Min Angle: {}, Range: {}".format(m_angle,m_range))
        # find max angles and max ranges and update front left and right arrays
        max_angle = max(front_angles, key=front_angles.get)
        max_range = front_angles[max_angle]
        right_angles = {}
        left_angles = {}
        for a, r in front_angles.items():
            if a > max_angle and r != max_range:
                left_angles[a] = r
            elif a < max_angle and r != max_range:
                right_angles[a] = r

        # find min of left and right arrays
        if right_angles and left_angles:
            right_min = right_angles[min(list(right_angles))]
            left_min = left_angles[min(list(left_angles))]

            # update closest range
            # self.prev_range = copy(self.closest_range)
            self.closest_range = min(front_angles.values())

            # update closest angle associated with closest_range
            for key, value in front_angles.items():
                if self.closest_range == value:
                    self.ang_vel_error = key

            a = min(front_angles,key=front_angles.get)
            x =  max(front_angles,key=front_angles.get)
<<<<<<< HEAD
            #rospy.logerr("{}".format(front_angles[x]))
            rospy.logerr("Min range: {}, min angle: {} max angle: {} range: {}".format(front_angles[a],a,x,front_angles[x]))
=======

            rospy.logerr("{}".format(front_angles[x]))
            #rospy.logerr("Min range: {}, min angle: {} max angle: {} range: {}".format(front_angles[a],a,x,front_angles[x]))
>>>>>>> d6873c23713a7bcb1ed5a0f4ade4fbb8d35d6259
            self.make_decision()

            # add all data to pickle file
            data['right'] = right_min
            data['left'] = left_min
            data['ranges']=front_angles
            self.add_to_file(data)

    # create pickle file and add data
    def add_to_file(self, data):
        saved_data = []
        if not path.exists(self.data_file_name):
            f = open(self.data_file_name, "wb+")
            f.close()
        else:
            saved_data = self.load_data()
            saved_data += [data]
        with open(self.data_file_name, 'wb') as fp:
            pickle.dump(saved_data, fp, protocol=pickle.HIGHEST_PROTOCOL)

    # loads data from file if needed
    def load_data(self):
        data_dict = []
        if path.exists(self.data_file_name) and path.getsize(self.data_file_name) > 0:
            with open(self.data_file_name, 'rb') as fp:
                try:
                    data_dict = pickle.load(fp)
                except Exception as e:
                    rospy.logerr("error: {}".format(e))
        return data_dict

    # decides state and updates error function
    def make_decision(self):
<<<<<<< HEAD
=======
        # if closest_range is within STOP_THRESHOLD then stop the robot
>>>>>>> d6873c23713a7bcb1ed5a0f4ade4fbb8d35d6259
        if self.closest_range < STOP_THRESHOLD:
            self.state = STOP
            #rospy.logerr("Stopping...")
            return

        # update error function for linear velocity
        self.prev_lin_vel = self.lin_vel_error
        e = self.closest_range - SAFETY_THRESHOLD
<<<<<<< HEAD
        vel = e * 2
        if vel > 2:
            vel = 2
        if vel < -2:
            vel = -2		
        self.prev_lin_vel = vel
        e_ang = self.ang_vel_error
        ang_vel = .25 * e_ang
        self.prev_ang_vel = ang_vel
=======
        vel = e * KP + (e-self.prev_lin_vel) * RATE * KD
        self.lin_vel_error = vel

        # update error function for angular velocity
        ang_vel = self.ang_vel_error
        e_ang_vel = .25 * ang_vel
        self.ang_vel_error = e_ang_vel

>>>>>>> d6873c23713a7bcb1ed5a0f4ade4fbb8d35d6259
        self.state = MOVE

    # publish the Twist message associated with the error functions
    def move_foward(self, lin_vel, ang_vel):
        vel_msg = Twist()
        vel_msg.linear.x = lin_vel
        vel_msg.angular.z = ang_vel
        rospy.logerr("linvel: {} angvel: {}".format(lin_vel, ang_vel))
        self.cmd_pub.publish(vel_msg)

    # controller for robot
    def spin(self):
        vel_msg = Twist()
        while not rospy.is_shutdown():
            # if state is STOP then send 0 angular and linear velocities
            if self.state == STOP:
               self.move_foward(0, 0)
            # if state is MOVE then send linear and angular velocities
            elif self.state == MOVE:
               self.move_foward(self.lin_vel_error, 0)

            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node("leader_follower", anonymous=True)
    lf = LeaderFollower()
    lf.spin()
