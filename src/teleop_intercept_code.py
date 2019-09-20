#!/usr/bin/env python

# roslaunch xml file: package, type = executable.py, name = node, elements
import roslib; roslib.load_manifest("teleop_intercept_package")
import rospy
import sys
import math
import copy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

global laser
global des_vel

def on_cmd(data):
	print("on_cmd")
        global des_vel
	
	des_vel = data

def on_lidar(data):
	global laser
        global pub
        global des_vel

        laser = data
        cmd_vel = copy.deepcopy(des_vel)
        
	numLasers = len(laser.ranges)
        print(des_vel.angular.z)
        cmd_vel.angular.z = 0.0
        print(des_vel.angular.z)
        cmd_vel.linear.x = des_vel.linear.x / 2.0
        
	rospy.loginfo("Distance to straight: " + str(data.ranges[135]))
	laser = data
	for n in range(int(numLasers)):
		angle = laser.angle_increment*n + laser.angle_min
		vscalar = -1/((50*laser.ranges[n])**5+1)
		cmd_vel.linear.x += 100000*vscalar*math.cos(angle)
		cmd_vel.angular.z += 2000000*vscalar*angle/(4*angle**6+1)
                

        cmd_vel.angular.z = cmd_vel.angular.z*abs(des_vel.linear.x) + des_vel.angular.z
                
	if cmd_vel.linear.x > 1:
		cmd_vel.linear.x = 1
	elif cmd_vel.linear.x < -1:
		cmd_vel.linear.x = -1
	if cmd_vel.angular.z > 6:
		cmd_vel.angular.z = 6
	elif cmd_vel.angular.z < -6:
		cmd_vel.angular.z = -6


        
	pub.publish(cmd_vel)

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
		print("Not enough args, using defaults")
		robot = "robot0"
		steering = "des_vel"
	else:
		robot = sys.argv[0]
		steering = sys.argv[1]
	try:
		intercept(robot, steering)
	except rospy.ROSInterruptException:
		pass
