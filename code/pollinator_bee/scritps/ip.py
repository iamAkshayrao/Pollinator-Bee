#!/usr/bin/env python
'''
* Team Id 		:1027
* Author List   :AKSHAY S RAO		    
* Filename      :ip.py
* Theme         :PB-POLLINATOR BEE
* Functions     : __init__(self),image_callback(self,msg)
		    
* Global Variables: NONE
'''

import rospy, cv2, cv_bridge
import numpy as np
from plutodrone.msg import *
from sensor_msgs.msg import	Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32
from std_msgs.msg import Float64

import rospy
import time
import datetime


#class imageprocessing

class imageprocessing:

'''
#* Function Name:   __init__(self)
#* Input:           reference to self,no input
#* Output:          NONE
#* Logic: 	   		initialises node,a publisher and a subscriber.converts image from camera to open cv frame.
#*
#* Example Call:	imageprocessing().
#'''	
	def __init__(self):

	 	rospy.init_node('ros_bridge')
	 	self.ros_bridge = cv_bridge.CvBridge()
		#publisher to publish processed image to /imageout topic
		self.image_pub = rospy.Publisher('/imageout', Image,queue_size=100000)

		#publishers for red,green and blue contours respectively
		self.red   = rospy.Publisher('/red', Int32,queue_size=10)
		self.green = rospy.Publisher('/green',Int32,queue_size=10)
		self.blue  = rospy.Publisher('/blue', Int32,queue_size=10)

		self.image_sub = rospy.Subscriber('/usb_cam/image_rect_color', Image,self.image_callback,queue_size=100)
		
		# initialising counts of pollinated flowers
		self.redcounts   = 0
		self.greencounts = 0
		self.bluecounts  = 0
		
	
	
'''
#* Function Name:   image_callback(self,msg)
#* Input:           image.
#* Output:          draws rectangles on detected LEDs
#* Logic: 	   the image obtained is assigned to a variable.this image is blurred and the lower and higher threshold values of the pixels
		       we want is found manually and is stored in param variable, then these values are used to mask the image which gives the output
		       of the LED color we want,on this mask contours are found,centroid of the obtained contour is obtained through moments,using this 
		       centroid value a rectangle is drawn on the boundary of the led.
		   
#*
#* Example Call: called through image_sub variable as a callback as self.image_callback
#'''
	
		
	def image_callback(self,msg):
		
		image1 = self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		image2 = self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		image3 = self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		image4 = self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

		#detects RED led
		#Mr is variable used to find moments of red
		#redcx= stores x coordinate of centroid of red led 
		#redcy= stores y coordinate of centroid of red led

		redimage = image1

		#Threshold for red
		red_threshold_low  = [131,120,163]				
		red_threshold_high = [145,137,213]

		lower_red = np.array(red_threshold_low)			#converted the threshold to numpy array	
		upper_red = np.array(red_threshold_high)

		red   	 = cv2.medianBlur(redimage,13)			# Applying blur	
		mask_red = cv2.inRange(red,lower_red,upper_red)	# Masking red color

		nullr, self.redcontours, nullr_1 = cv2.findContours(mask_red, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) # Finding contours

		self.redlength = len(self.redcontours) 			#calculating red flowers count

		#exception handling and drawing rectangle around detected pollinated flower
		try:
			if (self.redlength) == 1:
				Mr = cv2.moments(self.redcontours[-1]) #centroid calculation
				redcx = int(Mr['m10']/Mr['m00'])
				redcy = int(Mr['m01']/Mr['m00'])
	
				cv2.rectangle(image4,(redcx-20,redcy-15),(redcx+20,redcy+15),(0,0,255),1)
		except:
			pass

		#detects green led
		#Mg is variable used to find moments of green
		#greencx= stores x coordinate of centroid of green led 
		#greency= stores y coordinate of centroid of green led

		greenimage = image2

		green_threshold_low  = [130,170,140] 			#Threshold for green
		green_threshold_high = [165,210,157]

		lower_green = np.array(green_threshold_low) 	#converted the threshold to numpy array	
		upper_green = np.array(green_threshold_high)

		green      = cv2.medianBlur(greenimage,15)		# Applying blur	
		mask_green = cv2.inRange(green, lower_green, upper_green)

		nullg, self.greencontours, nullg_1 = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) # Finding contours

		self.greenlength = len(self.greencontours) 		#calculating red flowers count

		#exception handling and drawing rectangle for green pollinated flowers

		try:
			if (self.greenlength)==1:
				Mg = cv2.moments(self.greencontours[-1])
				greencx = int(Mg['m10']/Mg['m00'])
				greency = int(Mg['m01']/Mg['m00'])
	
				cv2.rectangle(image4,(greencx-20,greency-15),(greencx+20,greency+15),(0,255,0),1)
		except:
			pass

		#detects blue led
		#Mb is variable used to find moments of blue
		#bluecx= stores x coordinate of centroid of blue led 
		#bluecy= stores y coordinate of centroid of blue led

		blueimage = image3

		lower_threshold_blue = [236,185,150]
		upper_threshold_blue = [248,192,166]

		lower_blue = np.array(lower_threshold_blue)
		upper_blue = np.array(upper_threshold_blue)

		blue   	   = cv2.medianBlur(blueimage, 5)
		mask_blue  = cv2.inRange(blue, lower_blue, upper_blue)
		nullb, self.bluecontours, nullb_1 = cv2.findContours(mask_blue, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		self.bluelength = len(self.bluecontours)

		#exception handling and drawing rectangle on blue pollinated flowers
		try:
			if (self.bluelength)==1:
				Mb=cv2.moments(self.bluecontours[-1])
				bluecx=int(Mb['m10']/Mb['m00'])
				bluecy=int(Mb['m01']/Mb['m00'])
	
				cv2.rectangle(image4,(bluecx-20,bluecy-15),(bluecx+20,bluecy+15),(255,0,0),1)
		except :
			pass
		
		self.image_pub.publish(self.ros_bridge.cv2_to_imgmsg(image4,'bgr8'))
		self.red.publish(self.redlength)
		self.green.publish(self.greenlength)
		self.blue.publish(self.bluelength)
		
		
		
			
		
if __name__ == '__main__':
	while not rospy.is_shutdown():
		
		temp = imageprocessing()
		
		rospy.spin()
		
		
		
