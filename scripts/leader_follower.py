#!/usr/bin/python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
from os import path
from copy import copy
import pickle


INF = 20
PI = math.pi
SAFETY_THRESHOLD = 1.5
MOVE = 1
STOP = 0
# ANGLE_MIN = -7 * PI / 12.0
# ANGLE_MAX = 7 * PI / 12.0
ANGLE_MIN = 0
ANGLE_MAX = PI
RADIUS = 0.4


class LeaderFollower:
    def __init__(self):
        rospy.init_node("leader_follower", anonymous=True)
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=0)
        # self.scan_sub = rospy.Subscriber("base_scan", LaserScan, self.scan_callback, queue_size=1)
        self.scan_sub = rospy.Subscriber("scan", LaserScan, self.scan_callback, queue_size=1)
        self.closest_range = INF
        self.rate = rospy.Rate(100)
        self.state = 0
        self.prev_range = 0
        self.prev_lin_vel = 0.5
        self.prev_ang_vel = 0
        # self.angle_supplement = ANGLE_MAX - math.atan2(0.2, 1.5)  # TODO to set dx and dy as parameters
        self.angle_supplement = math.atan2(0.2, 1.5)
        self.ang_min = ANGLE_MIN - self.angle_supplement
        self.ang_max = self.angle_supplement
        # self.ang_max = ANGLE_MAX - self.angle_supplement
        self.data_file_name = "motion_data_{}.pickle".format(rospy.Time.now().secs)

    def scan_callback(self, msg):
        time_s = rospy.Time.now().secs
        data = {'time': time_s}
        ranges = msg.ranges
        resolution = msg.angle_increment
        min_ang = msg.angle_min
        front_angles = {}
        for i in range(len(ranges)):
            angle = min_ang + i * resolution
            if self.ang_min <= angle <= self.ang_max:
                front_angles[angle] = ranges[i]
        max_angle = max(front_angles, key=front_angles.get)
        max_range = front_angles[max_angle]
        right_angles = {}
        left_angles = {}
        for a, r in front_angles.items():
            if a > max_angle and r != max_range:
                left_angles[a] = r
            elif a < max_angle and r != max_range:
                right_angles[a] = r
        if right_angles and left_angles:
            right_min = right_angles[min(list(right_angles))]
            left_min = left_angles[min(list(left_angles))]
            self.prev_range = copy(self.closest_range)
            self.closest_range = min([left_min, right_min])
            self.make_decision()
            data['right'] = right_min
            data['left'] = left_min
            data['ranges']=front_angles
            self.add_to_file(data)

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

    def load_data(self):
        data_dict = []
        if path.exists(self.data_file_name) and path.getsize(self.data_file_name) > 0:
            with open(self.data_file_name, 'rb') as fp:
                try:
                    data_dict = pickle.load(fp)
                except Exception as e:
                    rospy.logerr("error: {}".format(e))
        return data_dict

    def make_decision(self):
        if self.closest_range < SAFETY_THRESHOLD:
            self.state = STOP
            return
        e = self.closest_range - SAFETY_THRESHOLD
        vel = self.prev_lin_vel + e * 10
        self.prev_lin_vel = vel
        self.prev_ang_vel = 0
        self.state = MOVE

    def move_foward(self, lin_vel, ang_vel):
        vel_msg = Twist()
        vel_msg.linear.x = lin_vel
        vel_msg.angular.z = ang_vel
        self.cmd_pub.publish(vel_msg)

    def spin(self):
        while not rospy.is_shutdown():
            if self.state == STOP:
                self.move_foward(0, 0)
            elif self.state == MOVE:
                # self.move_foward(2 * self.prev_lin_vel, self.prev_ang_vel)
                self.move_foward(2 * self.prev_lin_vel, self.prev_ang_vel)
            self.rate.sleep()


if __name__ == '__main__':
    lf = LeaderFollower()
    lf.spin()
