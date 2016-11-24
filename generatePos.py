#! /usr/bin/env python

#DESCRIPTION OF THIS NODE
#Generates starting position for the robot
#See one drive file for qandrant, coord & orientation explanation
#https://onedrive.live.com/redir?resid=ae64f94c6a5ff4f4!765&authkey=!AP7b1HnRxJyskMM&ithint=file%2cxlsx
#END OF THE DESCRIPTION

import random

def generateStart():
	largeQuad = random.randint(1, 19) #assume division of LG into 19 quadrants
	#coordinate of each quadrant
	smallX = random.randint(1,4) #x coord from 1 to 4	 
	smallY = random.randint(1,3) #y coord from 1 to 3
	orientation = random.randint(1, 8) #8 different orientations, separated by 45degrees 
	print ('Quadrant no %s, at (%s, %s), orientation no %s' % (largeQuad, smallX, smallY, orientation))

def generate10Start():
	for i in range(10):
		print ('Starting position #%s' % (i+1))
		generateStart()

if __name__ == '__main__':
	generateStart()
    #generate10Start()
