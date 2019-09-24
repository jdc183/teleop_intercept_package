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

# callback for receipt of des_vel message
def on_cmd(data):
	print("on_cmd")
        global des_vel
	
	des_vel = data

# Callback for receipt of laser data
def on_lidar(data):
	global laser
        global pub
        global des_vel

        laser = data
        cmd_vel = copy.deepcopy(des_vel) # Create a new Twist instance identical to des_vel
        
	numLasers = len(laser.ranges)
        print(des_vel.angular.z)
        cmd_vel.angular.z = 0.0	# Set rotational speed to zero (for now)
        print(des_vel.angular.z)
        cmd_vel.linear.x = des_vel.linear.x / 2.0 # Halve the linear speed so it's more manageable
        
	rospy.loginfo("Distance to straight: " + str(data.ranges[135]))
	laser = data

	# Modify linear and rotational velocities to avoid obstacles
	for n in range(int(numLasers)):
		angle = laser.angle_increment*n + laser.angle_min # calculate angle of each laser beam
		vscalar = -1/((50*laser.ranges[n])**5+1)	# A 5th order map that especially weights ranges under 0.1m or so
		cmd_vel.linear.x += 100000*vscalar*math.cos(angle) # Normalize the weighted range to the x direction for linear velocity
		cmd_vel.angular.z += 2000000*vscalar*angle/(4*angle**6+1) # Weight the ranges near front of fov for angular velocity
                
	# Adjusted rotational speed should be proportional to desired linear speed.  Finally add in the original desired angular velocity
        cmd_vel.angular.z = cmd_vel.angular.z*abs(des_vel.linear.x) + des_vel.angular.z
                
	# Don't let either velocity exceed reasonable bounds or the sim will stop working
	if cmd_vel.linear.x > 1:
		cmd_vel.linear.x = 1
	elif cmd_vel.linear.x < -1:
		cmd_vel.linear.x = -1
	if cmd_vel.angular.z > 6:
		cmd_vel.angular.z = 6
	elif cmd_vel.angular.z < -6:
		cmd_vel.angular.z = -6


        
	pub.publish(cmd_vel)

# Main
def intercept(robot, steering):
	print("intercept")
	global pub
	laser = LaserScan()
	print("cmd_vel: " + str(robot) + "/cmd_vel")
	print("des_vel: " + str(steering))
	print("laser:   " + str(robot) + "/laser_1")
	# Set up publisher and subscriber
	pub = rospy.Publisher(str(robot) + "/cmd_vel", Twist, queue_size=10)
	rospy.init_node("teleop_intercept_node", anonymous=True)
	telem_sub =  rospy.Subscriber(steering, Twist, on_cmd)
	lidar_sub = rospy.Subscriber(str(robot) + "/laser_1", LaserScan, on_lidar)
	rospy.spin()

if __name__ == "__main__":
	if len(sys.argv) < 3:
		print("Not enough args, using defaults")
		robot = "robot0"
		steering = "des_vel"
	else:	# set the topics we're interested in.  Note: sys.argv[0] is always this node's location or something, so we skip it.
		robot = sys.argv[1]
		steering = sys.argv[2]
	try:
		intercept(robot, steering)
	except rospy.ROSInterruptException:
		pass
