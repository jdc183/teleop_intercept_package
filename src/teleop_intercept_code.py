#!/usr/bin/env python
import roslib; roslib.load_manifest("teleop_intercept_package")
import rospy
import sys
import math
import copy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

global laser

def on_cmd(data):
	print("on_cmd")
	global pub
	gk = 0.05

	cmd_vel = copy.copy(data)
	numLasers = len(laser.ranges)

	for n in range(numLasers):
		angle = laser.angle_increment*n - laser.angle_min
		vscalar = gk/(laser.ranges[n]+1)**2
		cmd_vel.linear.x += vscalar*math.cos(angle)
		cmd_vel.linear.x += vscalar*math.sin(angle)
	
	print(cmd_vel.linear.x)
	pub.publish(cmd_vel)

def on_lidar(data):
	global laser
	rospy.loginfo("Distance to straight: " + str(data.ranges[135]))
	laser = data

def intercept(robot, steering):
	print("intercept")
	global pub
	laser = LaserScan()
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
