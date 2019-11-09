#!/usr/bin/python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from std_srvs.srv import Trigger, TriggerResponse
from time import sleep

INF = 10
PI = math.pi
SAFETY_THRESHOLD = 0.5
MOVE = 1
STOP = 0

DATA_FILE='data_file.pickle'
class Walk:
    def __init__(self):
        rospy.init_node("walk", anonymous=True)
        self.state = STOP
        self.make_request = 0
        self.initial = True
        self.rate = rospy.Rate(100)
        self.robot_id = rospy.get_param("~robot_id", -1)
        self.alternate_robot = rospy.get_param("~other_robot", -1)
        self.lin_vel = rospy.get_param("~linear_velocity", -1)
        self.ang_vel = rospy.get_param("~angular_velocity", -1)
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.move_srv = rospy.Service("move", Trigger, self.handle_start_stop_robot)
        rospy.wait_for_service('/robot_{}/move'.format(self.alternate_robot))
        self.move_client = rospy.ServiceProxy('/robot_{}/move'.format(self.alternate_robot), Trigger)

    def handle_start_stop_robot(self, req):
        if self.initial:
            self.move_forward(lin_vel=self.lin_vel, ang_vel=self.ang_vel)
            if self.robot_id < self.alternate_robot:
                sleep(1)
            else:
                sleep(4)
            self.move_forward()
            self.initial = False
        else:
            self.move_forward(lin_vel=self.lin_vel, ang_vel=self.ang_vel)
            sleep(2)
            self.move_forward()
        self.make_request = 1
        return TriggerResponse(True, "Robot state: {}".format(self.state))

    def move_forward(self, lin_vel=0.0, ang_vel=0.0):
        vel_msg = Twist()
        vel_msg.linear.x = lin_vel
        vel_msg.angular.z = ang_vel
        self.cmd_pub.publish(vel_msg)

    def spin(self):
        if self.robot_id < self.alternate_robot:
            self.move_client()
        while not rospy.is_shutdown():
            if self.make_request:
                self.move_client()
                self.make_request = 0
                sleep(2)


if __name__ == '__main__':
    lf = Walk()
    lf.spin()
