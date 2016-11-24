#! /usr/bin/env python

#DESCRIPTION OF THIS NODE
#Approach 3 - Left-Wall Spiral
#On initial move, robot moves forward until it finds a wall
#Rotate to have wall in its left-field of vision
#Move forward and turn appropriately, following the left wall
#Increase distance to wall after completing 1 round
#END OF THE DESCRIPTION

import rospy
import time
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)

# current approximate position of the robot
curX = 0 
curY = 0

# the starting position for this round
oriX = 0
oriY = 0

# used for timer
tick = 0
delay = 0
first_on = True

# distance to an obstacle/wall before robot needs to turn away or slow down (in metres)
hit_threshold = 0.44
slow_threshold = 0.6

# distance to the left to check for wall
turn_threshold = 0.55
turn_lower_threshold = 0.44

# error boundary to check if the robot makes one complete round
starting_radius = 0.3
	
def callback( sensor_data ):
	
	global hit_threshold,slow_threshold, turn_threshold, starting_radius
	n = len(sensor_data.ranges)
	base_data = Twist()
	
	hit = False
	slow = False
	global first_on, oriX, oriY, curX, curY,tick
	
	# check for obstacles that are directly blocking the way	
	for distance in sensor_data.ranges:
		if (distance < hit_threshold):
			hit = True
			if first_on:
				first_on = False
				tick = time.time()
				oriX = curX
				oriY = curY
				print "ORIGINAL POSITION %s,%s" %(oriX,oriY)
			break

	# check for nearby obstacles that might be blocking the way
	if not hit:
		for distance in sensor_data.ranges[n/3:n*2/3]:
			if (distance < slow_threshold):
				slow = True
				break
	
	# we should look for a wall first when initially turned on
	if first_on or hit:
		turning_left = False
	else:
		# look for an opening to the left
		turning_left = True
		for i in range(n/3,n):
			# keep a minimum distance from the wall
			if (sensor_data.ranges[i] < turn_lower_threshold):
				turning_left = False
				hit = True
				break
			elif (sensor_data.ranges[i] < turn_threshold):
				turning_left = False
				break
			
	# move the base properly
	if turning_left:
		#print "TURNING LEFT"
		base_data.linear.x = 0.1
		base_data.angular.z = 0.25
	elif hit:
		#print "AVOIDING SOMETHING"
		base_data.linear.x = 0
		base_data.angular.z = -0.3
	elif slow:
		#print "MOVING FORWARD SLOWLY"
		base_data.linear.x = 0.2
		base_data.angular.z = 0
	else:
		#print "MOVING FORWARD QUICKLY"
		base_data.linear.x = 0.35
		base_data.angular.z = 0
	pub.publish(base_data)

def odometry_callback(msg):
	global turn_lower_threshold,turn_threshold, starting_radius
	global curX,curY,oriX,oriY,tick,delay
	curX  = msg.pose.pose.position.x
	curY  = msg.pose.pose.position.y
	
	if first_on:
		return
	# a delay time is used to find a new original point
	if delay>0:
		if time.time()-tick<delay:
			return
		else:
			oriX = curX
			oriY = curY
			print "NEW ORIGINAL POSITION %s,%s" %(oriX,oriY)
			delay = 0
			time.time()
		# increase the distance to left wall to discover the inside (or outside) area
	elif time.time()-tick > 20 and math.fabs(curX-oriX)<starting_radius and math.fabs(curY-oriY)<starting_radius:#detect that robot has completed a whole round of the room
		print "I WENT IN A CIRCLE!! SPIRALING IN!!!"
		turn_lower_threshold += 0.3
		turn_threshold += 0.3
		tick = time.time()
		delay = 4

if __name__ == '__main__':
    rospy.init_node('move')
    base_data = Twist()
    rospy.Subscriber('odom', Odometry, odometry_callback)
    rospy.Subscriber('base_scan', LaserScan, callback)
    rospy.spin()
