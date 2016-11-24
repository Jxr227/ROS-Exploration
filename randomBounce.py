#! /usr/bin/env python

#DESCRIPTION OF THIS NODE
#Approach 1 - Random bounce
#Robot moves forward until it detects an obstacle in front of it 
#The robot turns by a random value
#and repeat process above
#END OF THE DESCRIPTION

import rospy
import time
import random
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
tick = 0
turning = 0
turning_time = 5
obs_Threshold = 0.3 #distance to an obstacle/wall before robot needs to turn away (in metres)
slow_threshold = 0.5 #distance to an obstacle/wall before robot needs to slow down (in metres)

def callback( sensor_data ):
	global obs_Threshold,tick, turning, turning_time
	n = len(sensor_data.ranges) 
	base_data = Twist()
	hit = False #variable to check for obstacle
	speedFactor = 1
	# check for obstacles in front of the robot	
	if turning == 0:
		for i in range(0,n):
			d = sensor_data.ranges[i] #sensor reading at index i 
			#robot vision divided into 3
			#if i<n/3 then it's a reading in robot's left-field
			#if i>n*2/3 then it's a readding in robot's right-field			
			
			if i>n/3 and i<n*2/3 and speedFactor==1 and d < slow_threshold: #sensor reading is in middle-field (front) and near an obstacle
				speedFactor = 0.5 #slow down
			if (d < obs_Threshold): #obstacle is detected
				hit = True
				tick = time.time()
				if (i<n/2):
					turning = 1
				else:
					turning = -1
				turning_time = 5 + 6*(random.random()-0.5) #generate random value for turn (in terms of time)
				break
			
	# move the robot properly
	if turning != 0:
		if time.time()-tick>turning_time:
			turning = 0
		else: 
			base_data.linear.x = 0
			base_data.angular.z = 0.3*turning #turn by random amount
	elif hit:
		print "I HIT SOMETHING"
	else:
		base_data.linear.x = 0.4*speedFactor 
		base_data.angular.z = 0
	pub.publish(base_data)

if __name__ == '__main__':
    rospy.init_node('move')
    base_data = Twist()
    global tick
    tick = time.time()
    rospy.Subscriber('base_scan', LaserScan, callback)
    rospy.spin()
