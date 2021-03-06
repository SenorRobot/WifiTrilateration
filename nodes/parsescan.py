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
#
# March:
# Added Stuff
#
# 4/5/12:
# Started adding transform integration to get location from robot

import os
import string
import roslib; roslib.load_manifest('wifiScanner')
import rospy
import tf
from std_msgs.msg import String
import random

def scanLoop():
	#Start Ros loop
	pub=rospy.Publisher('aps',String)
	rospy.init_node('wifiTalker')
	#Transform Listener
	listener = tf.TransformListener()
	rate = rospy.Rate(.1) #10 hertz

	while not rospy.is_shutdown():	
	#Get current location from vehicle
	#store in 3 ints (x,y,z)
		x=0.0
		y=0.0
		z=0.0
	#Get from vehicle
		try:
			listener.waitForTransform('/wifiAntenna','/map',rospy.Time(0), rospy.Duration(3))
			(trans,rot) = listener.lookupTransform('/wifiAntenna','/map',rospy.Time(0))
			print("started!\n")
			print trans
			x=trans[0]
			y=trans[1]
			z=trans[2]
		except (tf.LookupException, tf.ConnectivityException, tf.Exception):
			continue
		
	#Run scan of all wireless access points
		#os.system("sudo wpa_cli scan") #force refresh of scan
		#os.system("sudo wpa_cli scan_results > scanout.txt")
		os.system("sudo iwlist wlan1 scanning | python iwlistparse.py > scanout.txt")
		#Parse the output
		#open file into lines
		f = open('scanout.txt')
		fout = open('testout.txt','w')
		counter=0
		for line in f:
		#skip first two lines - they are headers of the table
			if(counter>=1): 
			#We need the Mac Address, and signal trength
			#Mac address is first parameter in each line, signal strength is third
				lineInfo=line.split()
				#convert signal strength into distance
				#if '00:22:90:95:f4:b0' in lineInfo[0]: #this is just to test the estimations of distance to calibrate
				#	print lineInfo[0] #MAC Address
				#	print lineInfo[2] #Signal
				#	print lineInfo[4] #BSSID
				mac=lineInfo[0]
				signal=int(lineInfo[2])
				#print "Signal {0} = {1}m".format(lineInfo[2],signal) #Test output
				ssid=lineInfo[4]
				#format and publish to ros topic
				str="{0}\t{1}\t{2}\t{3}\t{4}\t{5}".format(mac,signal,x,y,z,ssid)
				pub.publish(String(str))
				fout.write(str)
			else: counter+=1
		pub.publish(String("END_SCAN"))
		#Just for testing, replace with distance/speed based loop interval		
		#rospy.sleep(1.0)
		rate.sleep();

if __name__ =='__main__':
	try:
		scanLoop()
	except rospy.ROSInterruptException:pass
