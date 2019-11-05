#!/usr/bin/python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def scan_callback(data):
    pass


if __name__=='__main__':
    rospy.init_node("localizer",anonymous=True)
    cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=0)
    scan_sub = rospy.Subscriber("base_scan", LaserScan, scan_callback, queue_size=1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()

