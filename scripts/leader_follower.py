#!/usr/bin/python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

INF = 1000
PI = math.pi
SAFETY_THRESHOLD = 0.5
MOVE = 1
STOP = 0


class LeaderFollower:
    def __init__(self):
        rospy.init_node("leader_follower", anonymous=True)
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=0)
        self.scan_sub = rospy.Subscriber("scan", LaserScan, self.scan_callback, queue_size=1)
        self.front_left = INF
        self.front_right = INF
        self.rate = rospy.Rate(100)
        self.state = 0
        self.initial_distance = 0
        self.prev_lin_vel = 0.5
        self.prev_ang_vel = 0

    def scan_callback(self, msg):
        ranges = msg.ranges
        ang_min = msg.angle_min
        ang_max = msg.angle_max
        resolution = msg.angle_increment
        front_left = INF
        front_right = INF
        for i in range(len(ranges)):
            angle = ang_min + i * resolution
            if ang_min + PI / 2 <= angle <= ang_max - PI / 2:
                if ranges[i] < front_left:
                    front_left = ranges[i]
            if ang_min <= angle < ang_min + PI / 6:
                if ranges[i] < front_right:
                    front_right = ranges[i]
        self.front_left = front_left
        self.front_right = front_right
        self.make_decision()

    def make_decision(self):
        rospy.logerr(self.front_left)
        if self.front_left < SAFETY_THRESHOLD:
            self.state = STOP
            return
        diff = self.front_left - self.initial_distance
        vel = self.prev_lin_vel + diff * 10
        self.prev_lin_vel = vel
        self.prev_ang_vel = 0
        self.initial_distance = self.front_left
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
                self.move_foward(self.prev_lin_vel, self.prev_ang_vel)
            self.rate.sleep()


if __name__ == '__main__':
    lf = LeaderFollower()
    lf.spin()
