#!/usr/bin/env python
import roslib; roslib.load_manifest("teleop_intercept_package")
import rospy
import sys
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

global laser

def on_cmd(data):
	rospy.loginfo("forward velocity: " + str(data.linear.x))

def on_lidar(data):
	rospy.loginfo("Distance to straight: " + str(data.ranges[135]))

def intercept(robot, steering):
	pub = rospy.Publisher(str(robot) + "/cmd_vel", Twist, queue_size=10)
	rospy.init_node("teleop_intercept_node", anonymous=True)
	telem_sub =  rospy.Subscriber(steering, Twist, on_cmd)
	lidar_sub = rospy.Subscriber(str(robot) + "/laser_1", LaserScan, on_lidar)
	rospy.spin()

if __name__ == "__main__":
	if len(sys.argv) < 2:
		print("Not \'nuff args, using defaults")
		robot = "robot0"
		steering = "des_vel"
	else:
		robot = sys.argv[0]
		steering = sys.argv[1]
	try:
		intercept(robot, steering)
	except rospy.ROSInterruptException:
		pass
