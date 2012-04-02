#!/usr/bin/env python

#Parsescan.py
#Russ Martin
#2/9/12
#Gets current location of vehicle, scans all wireless access points,
# and parses commands to store in a data structure that is used by
# calculateap.py to measure estimated locations of access points
# and the vehicle
#
#Edit Log:
# 2/15/12:
# Added Tab Based Parsing
# Started Export to Calculating File

import os
import string
import roslib; roslib.load_manifest('wifiScanner')
import rospy
from std_msgs.msg import String
import random

def scanLoop():
	#Start Ros loop
	pub=rospy.Publisher('aps',String)
	rospy.init_node('wifiTalker')
	while not rospy.is_shutdown():	
	#Get current location from vehicle
	#store in 3 ints (x,y,z)
		x=0.0
		y=0.0
		z=0.0
	#TESTING: Randomize x,y from -20 to 20
		x=random.random()
		if(x<0.5):
			x*=-40
		else:
			x-=0.5
			x*=40
		y=random.random()
		if(y<0.5):
			y*=-40
		else:
			y-=0.5
			y*=40
	#Run scan of all wireless access points
		os.system("sudo wpa_cli scan") #force refresh of scan
		os.system("sudo wpa_cli scan_results > scanout.txt")
		#Parse the output

		#open file into lines
		f = open('scanout.txt')
		counter=0
		for line in f:
		#skip first two lines - they are headers of the table
			if(counter>=2): 
			#We need the Mac Address, and signal trength
			#Mac address is first parameter in each line, signal strength is third
				lineInfo=line.split()
				#convert signal strength into distance
				if '00:22:90:95:f4:b0' in lineInfo[0]: #this is just to test the estimations of distance to calibrate
					print lineInfo[0] #MAC Address
					print lineInfo[2] #Signal
					print lineInfo[4] #BSSID
				mac=lineInfo[0]
				signal=int(lineInfo[2])
				#signal strength: Current calibration is -39 - (3*m)
				if signal >=-39:
					signal=0 #above this dbm is assumed to be right at the AP
				else:
					signal = ((-1 * signal) - 39)/3
				#print "Signal {0} = {1}m".format(lineInfo[2],signal) #Test output
				ssid=lineInfo[4]
				#format and publish to ros topic
				str="{0}\t{1}\t{2}\t{3}\t{4}\t{5}".format(mac,signal,x,y,z,ssid)
				pub.publish(String(str))
		
			else: counter+=1
		#Just for testing, replace with distance/speed based loop interval		
		rospy.sleep(10.0)

if __name__ =='__main__':
	try:
		scanLoop()
	except rospy.ROSInterruptException:pass
