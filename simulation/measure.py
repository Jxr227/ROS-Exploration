#! /usr/bin/env python

#DESCRIPTION OF THIS NODE
#Measures the performance of an approach, to determine which is best
#Measures time, distance and coordinates visited until the robot is stuck
#Also prints measurements recorded in each minute
#Displays total measurements every 5 minutes
#END OF THE DESCRIPTION

import rospy
import time
import math
from nav_msgs.msg import Odometry
from math import sqrt
from sets import Set

t0 = 0 #starting time 
prevX = 0 #previous x position
prevY = 0 #previous y position
dist = 0 
currX = 0 #current x position
currY = 0 #current y position
minuteCounter = 0 
duplicateCounter = 0
distThresh = 0 #distance variable to record coordinate
visitedCoord = Set()
prevMinuteDist = 0 #total distance travelled, excluding distance travelled in the current minute
prevUCoord = 0 #total unique coordinates visited, excluding current minute
prevDCoord = 0 #total unique coordinates visited, excluding current minute

#Obtains robot's position
def odometry_callback(msg):
	global prevX, prevY, dist, currX, currY
	currX  = msg.pose.pose.position.x #get X coordinate of current position
	currY  = msg.pose.pose.position.y #get Y coordinate of current position
	if (prevX != 0) and (prevY != 0): #ignore first coord
		calcDist()
	prevX = currX
	prevY = currY
	checkEveryMin()

def checkEveryMin():
	global t0, minuteCounter, dist, prevMinuteDist, prevUCoord, prevDCoord, duplicateCounter
	temp = time.time()
	diff = round(temp - t0, 0) 
	#check if 1 minute has passed
	if diff - (minuteCounter * 60) == 60.0: 
		minuteCounter += 1
		#calculate measurements for the last minute
		totalDist = round(dist, 2)
		minuteDist = totalDist - prevMinuteDist
		totalUCoord = len(visitedCoord)
		minuteCoord = totalUCoord - prevUCoord
		minuteDCoord = duplicateCounter - prevDCoord
		print ('For Minute no. %s' % minuteCounter)
		print ('Distance: %s m, Unique coord: %s, Duplicate coord: %s' % (minuteDist, minuteCoord, minuteDCoord))
		#prints total measurements every 5 minutes
		if minuteCounter % 5 == 0 :
			print ('----------------Total measurement after %s minutes----------------' % minuteCounter)
			print ('Total Distance: %s m, Total Unique coord: %s, Total Duplicate coord: %s' % (totalDist, len(visitedCoord), duplicateCoord))
		prevMinuteDist = totalDist
		prevUCoord = totalUCoord
		prevDCoord = duplicateCounter

#calculate distance between 2 points (prevX, prevY) and (currX, currY)
def calcDist():
	global prevX, prevY, dist, currX, currY, distThresh
	xDiff = (prevX - currX)
	yDiff = (prevY - currY)
	currDist = sqrt((xDiff**2) + (yDiff**2))
	if currDist > 0:
		dist += currDist
		distThresh += currDist
		if distThresh > 0.5: #record coordinate every 0.5 metres travelled
			recCoord()
			distThresh = 0

#Save coordinates visited in a set, record no. of duplicate coordinates visited
def recCoord():
	global duplicateCounter, visitedCoord
	coord = (roundToNextHalfMetre(currX), roundToNextHalfMetre(currY))
	#check if new coordinate has not been visited before
	if coord not in visitedCoord: 
		visitedCoord.add(coord)
	else: 
		#print 'UH OH. I\'ve been here before'
		duplicateCounter += 1
		
def roundToNextHalfMetre(value):
	unit = math.floor(value)
	temp = value - unit
	if temp - 0.5 > 0:
		value = unit + 1
	else:
		value = unit + 0.5
	return value

#print total distance (after robot is stuck)	
def stopDistance():
	totalDist = round(dist, 2)
	print('Total distance is %s metres' % totalDist)

#starts robot's timer
def startTimer():
	global t0
	t0 = time.time()

#print total time travelled (after robot is stuck)	
def stopTimer():
	global t0
	t1 = time.time()
	totalTime = round(t1 - t0, 2)
	print('Total time is %s seconds' % totalTime)

#print total unique and duplicate coordinates visited (after robot is stuck)		
def stopMeasuring():	
	global duplicateCounter, visitedCoord
	stopTimer()
	stopDistance()
	print ('Visited a total of %s unique coordinates' % len(visitedCoord))
	print ('Visited duplicate coordinates %s times' % duplicateCounter)	
	

def calculate():
	rospy.init_node('calculate_distance')
	start = raw_input('Prompt to start:' )
	startTimer()	
	rospy.Subscriber('odom', Odometry, odometry_callback)
	stop = raw_input('Prompt to stop: ') #stop measurements when robot is visibly stuck
	stopMeasuring()
	rospy.spin()

if __name__ == '__main__':
	calculate()
