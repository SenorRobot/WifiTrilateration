#!/usr/bin/env python
import roslib; roslib.load_manifest('wifiScanner')
import rospy
from std_msgs.msg import String
import numpy as np
import scipy.optimize as optimize
from pylab import *


#global dictionary used here
apData = dict()
def callback(data):
	#this means data has been obtained from the listener
	#parse data into components and place in dictionary
	dataInfo=data.data.split()
	mac=dataInfo[0]
	dataTuple=tuple(dataInfo[1:6])
	#dataTuple is in format
	#(signalStrength, x, y ,z, bssid)
	#global apData
	if mac in apData:
		#data for key/mac address exists
		apData[mac].append(dataTuple)
	else:
		#mac address is new
		apData[mac]=[dataTuple]
	#print apData[mac]
	#now perform the analysis on each of the ap entries
	pointNum=len(apData[mac])
	calcAPLocation(mac, pointNum)

def loadFileData(filename):
	#loads data points from a file instead of from ros topic
	fin=open(filename)
	for data in fin:
		#this means data has been obtained from the listener
		#parse data into components and place in dictionary
		dataInfo=data.split()
		mac=dataInfo[0]
		dataTuple=tuple(dataInfo[1:6])
		#dataTuple is in format
		#(signalStrength, x, y ,z, bssid)
		#global apData
		if mac in apData:
			#data for key/mac address exists
			apData[mac].append(dataTuple)
		else:
			#mac address is new
			apData[mac]=[dataTuple]
		#print apData[mac]
		#now perform the analysis on each of the ap entries
		pointNum=len(apData[mac])
		calcAPLocation(mac, pointNum)

#Calculate the Location of the access point with MAC Address mac
def calcAPLocation(mac,pointNum):
	print "Starting Calculation of {}".format(mac)
	#if there are more than 2 data points for each AP, then we can perform the calculation
	#print "{0}\t{1}".format(mac,len(apData[mac]))
	if pointNum>=2:
		#average the centers of the circles
		sumX = 0.0
		sumY = 0.0
		#Also get all xi, yi, and radii for this access point in arrays
		#They will be needed for least squares, allows a single loop through
		xi = []
		yi = []
		radii = []
		for dataPt in apData[mac]:
			sumX+=float(dataPt[1])
			sumY+=float(dataPt[2])
			xi.append(float(dataPt[1]))
			yi.append(float(dataPt[2]))
			radii.append(float(dataPt[0]))
		avgX = sumX/pointNum
		avgY = sumY/pointNum
		#print "Average X: {0}\tY:{1}".format(avgX,avgY)
		#Now that we have the center, we can do least squares
		#generate point guess starting at avg of circles
		ptGuess = np.array([avgX,avgY])
		#Convert arrays to type used by least squares
		#xi_p = np.array(xi)
		#yi_p = np.array(yi)
		#radii_p = np.array(radii)				
		point= optimize.leastsq(calcResiduals, ptGuess, args = (xi,yi,radii))
		print point
	print "\n"	
	print "End of Calculation"


#Function that calculates and generates the residuals for a function
def calcResiduals(ptGuess, xi, yi, radii):
	#extract x and y from guess point
	xg = ptGuess[0]
	yg = ptGuess[1]
	#slope of the line from (xi,yi) to guess (xg,yg)
	m = (yg - yi) / (xg - xi)
	#Go along the line for the distance of c to get coordinates
	deltax = radii / np.sqrt(1+m**2)
	xii = []
	yii = []
	for i in range(0, len(xi)):		
		if (xi[i] < xg):
			xii.append(xi[i] + deltax[i])
		else:
			xii.append(xi[i] - deltax[i])
		yii.append(m[i]*(xii[i]-xi[i]) + yi[i])
	#residuals is distance from (xii,yii) to (xg, yg)
	return (xii-xg)**2 + (yii-yg)**2	

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("aps", String, callback)
    rospy.spin()

if __name__ == '__main__':
    #listener()
	loadFileData("test1.txt")
