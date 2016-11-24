#! /usr/bin/env python

#DESCRIPTION OF THIS NODE
#Approach 2 - Follow Max Distance
#The robot's vision is divided into 3 sections (left-field, direct front, right-field)
#Get average of all sensor readings in each section
#Robot will move forward or turn towards section with highest average reading
#END OF THE DESCRIPTION

import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
move = Twist()
obs_Threshold = 0.3 #distance to an obstacle before robot needs to turn away (in metres)
slow_Threshold = 0.5 #distance to an obstacle/wall before robot needs to slow down (in metres)
speedFactor = 1

#move robot to turn left
def turnLeft(): 
	print 'left'
	move.linear.x = 0.3*speedFactor #slow down on turn
	move.angular.z = 0.3

#move robot to turn right
def turnRight():
	print 'right'
	move.linear.x = 0.3*speedFactor #slow down on turn
	move.angular.z = -0.3
	
#move robot forward
def moveForward():
	print 'forward'
	move.angular.z = 0 
	move.linear.x = 0.4*speedFactor

def checkForObs(array): #check for obstacle, compare sensor reading to obs_Threshold
	global front, left, right
	oLeft = False
	oRight = False
	slow = False
	for i in range(0,len(array)):
		value = array[i]
		if value < obs_Threshold:
			print 'found obs'
			#turn left & move forward
			if (i>(len(array)/2)):
				print "obstacle on the left side"
				oLeft = True
			else:
				print "obstacle on the right side"
				oRight = True 
	if oLeft: #detected obstacle on left side
		side = -1
	elif oRight: #detected obstacle on right side
		side = 1
	else:
		return False
	move.linear.x = 0
	move.angular.z = side*0.3
	pub.publish(move)
	return True
			#move.angular.z = -1

def checkForFarObs (array): 
	global speedFactor
	n = len(array)
	for distance in array:
		if distance < slow_Threshold: #slow down when getting close to an obstacle, prevent crashing
			speedFactor = 0.5
			print "slowing down!"
			return True
	speedFactor = 1
	return False

def decideMove(right, front, left):
	diff = left - right
	diffThreshold = 0.4
	if front >= left and front >= right: #forward is max
		moveForward()
	elif math.fabs(diff) < diffThreshold:
		moveForward()
	elif diff < 0 : #right is max
		turnRight()
	elif diff > 0: #left is max
		turnLeft()
	pub.publish(move)

def scan_callback(scan):
	global totalAngle
	global front, left, right
	reading = scan.ranges
	totalAngle = len(reading)
	if not checkForObs(reading): #make sure robot doesn't crash during a move
		right = getAverage(reading[:totalAngle/3])
		front = getAverage(reading[totalAngle/3:totalAngle*2/3])
		left = getAverage(reading[totalAngle*2/3:]) 
		print "left %s, front %s, right %s" %(left,front,right)
		checkForFarObs(reading)
		decideMove(right, front, left)

def getAverage(array,inf_sub=5):
	n = 0
	s = 0
	for val in array:
		if not math.isnan(val): #disregard 'not a number' - considered as noise in sensor reading
			if math.isinf(val): #handles 'infinite' reading, 
				s+= inf_sub
			else:
				s+= val
			n+= 1
	return s/n

def listener():
	rospy.init_node('follow_Max_Distance')
	rospy.Subscriber('base_scan', LaserScan, scan_callback)
	rospy.spin()

if __name__ == '__main__':
	listener()
