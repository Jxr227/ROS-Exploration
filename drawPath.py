#! /usr/bin/env python

#DESCRIPTION OF THIS NODE
#This node draws a representation of the path the robot thinks it has been through
#END OF THE DESCRIPTION

import rospy
from nav_msgs.msg import Odometry
from math import sqrt
import turtle
import time

#Adjusts the scale of the path drawing
TURTLE_SCALE=float(0.1)
#The distance between 2 pts on the grid, in m
GRID_UNIT=0.01

#Variables for the timer
timeStart = time.time()
timeEnd = time.time()
minute = 0

initialised=False

#Main callback method
def odometry_callback(msg):
	initialise()
	currX  = round(msg.pose.pose.position.x / TURTLE_SCALE)
	currY  = round(msg.pose.pose.position.y / TURTLE_SCALE)
	
	drawNewPoint(currX,currY)
	writeTime()

#Starts the Turtle screen (representation of the path)
#This method must only be called by the callback from the odometry, not by the main method
def initialise():
	global initialised		
	if not initialised:		
		turtle.begin_fill()
		initialised =True
			
#Draws a point on Turtle
def drawNewPoint(X,Y):		
	turtle.goto(X,Y)

def writeTime():
	#If one minute has elapsed since the last time record, write the time on Turtle
	global timeEnd, timeStart, minute
	timeEnd = time.time()
	if (timeEnd - timeStart) >= 60.0:
		minute += 1
		#print(str(minute) + " minutes passed")
		oldColor=turtle.pencolor()
		turtle.pencolor("red")
		turtle.write(str(minute),False, "Right")
		turtle.pencolor(oldColor)
		timeStart = time.time() 

if __name__ == '__main__':
	print "main"
	rospy.init_node('drawMap')
	rospy.Subscriber('odom', Odometry, odometry_callback)
	rospy.spin()

