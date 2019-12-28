#!/usr/bin/env python
'''
* Team Id 		: 1027
* Author List 	: AKSHAY S RAO		    
* Filename		: pos_hold.py
* Theme			: PB-POLLINATOR BEE
* functions		: arm(self),disarm(self), __init__(self), position_hold(self), calc_pid(self), 
		    	  pid_yaw(self), pid_roll(self), pid_pitch(self), pid_throt(self), limit(self, int, int, int)
		          get_pose(self,pose), get_t(self,Float),get_red(self,Int),get_green(self,Int),get_blue(self,Int)
		    
* Global Variables: NONE

'''
#The required packages are imported here
from sensor_msgs.msg import	Image
from plutodrone.msg import *
from pid_tune.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32 
from std_msgs.msg import Float64

import rospy, cv2, cv_bridge
import time
import numpy as np



class DroneFly():


	def __init__(self):
		
		rospy.init_node('pluto_fly', disable_signals = True)
		self.ros_bridge = cv_bridge.CvBridge()
		
		

		self.pluto_cmd = rospy.Publisher('/drone_command', PlutoMsg, queue_size=10)

		rospy.Subscriber('whycon/poses', PoseArray, self.get_pose)
		rospy.Subscriber('/drone_yaw', Float64,self.get_t)

		#subscribers for contours of red green and blue plant respectively
		self.red_sub   = rospy.Subscriber('/red', Int32, self.get_red)
		self.green_sub = rospy.Subscriber('/green', Int32, self.get_green)
		self.blue_sub  = rospy.Subscriber('/blue', Int32, self.get_blue)
		
		#publishers for error in altitude,roll,pitch and yaw
		self.pub  = rospy.Publisher('alt_err', Float64, queue_size=10)
		self.pub1 = rospy.Publisher('row_err', Float64, queue_size=10)
		self.pub2 = rospy.Publisher('pitch_err', Float64, queue_size=10)
		self.pub3 =r ospy.Publisher('yaw_error',Float64, queue_size=10)

		
		
		self.cmd = PlutoMsg()


		#stores the value of coordinates of plants
		self.Plant_location = 	[(-2.0, 1.0, 25.0),(4.8, -5.5, 25.0) , (4.8,-5.5, 25.7), (4.8, -5.5, 25.0),
								 (-1.0, -5.8, 25.0), (-1.0,-5.8,29.5), (-1.0, -5.8, 28.0),
								 (-6.0, -2.8, 28.0), (-6.0,-2.8,29.5), (0.0, 0.0, 25.0) ]
		
		#intitialising an iterator which picks the right movement of drone in change_self_iter function 
		self.iter=0

		#initialising the contour value of red,green and blue respectively	
		self.redcounts=0
		self.greencounts=0
		self.bluecounts=0
		
		

		# Position to hold.
		
		
		(self.wp_x,self.wp_y,self.wp_z)=(-2.0, 1.0, 25.0)
				
		self.wp_t = 0.0
		
		#intitalising the drone commands.
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1000
		self.cmd.plutoIndex = 0

		#intialising the initial position of the drone

		self.drone_x = 0.0
		self.drone_y = 0.0
		self.drone_z = 40.0
		self.drone_t = 0.0
		

		#PID constants for Roll
		self.kp_roll = 10.0
		self.ki_roll = 0.083
		self.kd_roll = 21.25

		#PID constantsr Pitch
		self.kp_pitch = 7.8
		self.ki_pitch = 0.075
		self.kd_pitch = 30.0
		
		#PID constants for Yaw
		self.kp_yaw = 8.0
		self.ki_yaw = 0.0
		self.kd_yaw = 5.0

		#PID constants for Throttle
		self.kp_throt = 29.0
		self.ki_throt = 0.085
		self.kd_throt = 4.3

		

		# Correction values after PID is computed
		self.correct_roll  = 0.0
		self.correct_pitch = 0.0
		self.correct_yaw   = 0.0
		self.correct_throt = 0.0

		# Loop time for PID computation. 
		self.last_time = 0.0
		self.loop_time = 0.032

		#intialising errors 
		self.throt_previous_error = 0.0
		self.yaw_previous_error   = 0.0
		self.pitch_previous_error = 0.0
		self.roll_previous_error  = 0.0

		#intialising integral errors.
		self.yaw_iterm   = 0.0
		self.roll_iterm  = 0.0
		self.pitch_iterm = 0.0
		self.throt_iterm = 0.0
		self.count = 0
		
		

		rospy.sleep(.1)


	
"""
* Function Name:   arm(self)
* Input:	   reference to self,no input
* Output:	   publishes rcThrottle value as 1000 ,rcAUX4 value as 1500
* Logic:	   none	
*
* Example Call:    called by function position_hold(self) as self.arm()


"""

	def arm(self):
		self.cmd.rcAUX4 = 1500
		self.cmd.rcThrottle = 1000
		self.pluto_cmd.publish(self.cmd)
		rospy.sleep(.1)



"""
* Function Name:   disarm(self)
* Input:	   reference to self,no input
* Output:	   publishes rcAUX4 = 1100
* Logic:	   none
*
* Example Call:    called by function position_hold(self) as self.disarm()


"""
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.pluto_cmd.publish(self.cmd)
		rospy.sleep(.1)


"""
* Function Name:  position_hold(self) 
* Input:	  reference to self,no input
* Output:	  publishes the value of rcPitch,rcRoll,rcThrottle,rcYaw
* Logic:	  subtracts or adds the correct values of pitch,roll,throttle and yaw
*
* Example Call:	  temp.position_hold()


"""

	def position_hold(self):

		rospy.sleep(2)

		print "disarm"
		self.disarm()
		rospy.sleep(.2)
		
		print('START')
		print "arm"
		self.arm()
		rospy.sleep(.1)
		

		while True:
			
			self.calc_pid()

			
		 	pitch_value 	 = int(1500 - self.correct_pitch)
			self.cmd.rcPitch = self.limit (pitch_value, 1600, 1400)
			
															
			roll_value 		= int(1500 + self.correct_roll)
			self.cmd.rcRoll = self.limit(roll_value, 1600,1400)
															
			
			yaw_value 		= int(1500 - self.correct_yaw)
			self.cmd.rcYaw  = self.limit(yaw_value, 1600,1400)

			throt_value 		= int(1500 - self.correct_throt)
			self.cmd.rcThrottle = self.limit(throt_value, 1750,1350)
			
			self.change_self_iter()

			#self.publish_plot_data()		
															
			self.pluto_cmd.publish(self.cmd)

'''
* Function Name : change_self_iter(self)
* Input         : NONE
* Output        : assigns the next coordinate
* Logic         : checks whether the drone is in within the error limit,if it is within error limit
		          it will increment iter value by 1.for iter value 2,5 and 8 it will go for touching the plant,
		          for iter value 1,4 and 7 it will go above the plants after touching the plant ,it will check 
		          for the length of the contours for respective plants,if its not greater than or equal 1,it will 
		          again go to touch the plant.
*
* Example Call:	   self.change_self_iter()
'''	

	def change_self_iter(self):
		self.alt_err_data =self.wp_z-self.drone_z
		self.pitch_err_data=self.wp_x-self.drone_x
		self.roll_err_data=self.wp_y-self.drone_y
		
		if ((-1.0<=self.alt_err_data<=1.0)and  (-2.0<=self.roll_err_data<=2.0) and (-2.0<=self.pitch_err_data<=2.0)):
					
			self.iter+=1	
			if ((self.iter == 3) and (self.bluecounts < 1)):
				self.iter -= 1
				(self.wp_x,self.wp_y,self.wp_z) = (self.Plant_location[self.iter][0], 
												   self.Plant_location[self.iter][1], 
												   self.Plant_location[self.iter][2])	
				
			elif self.iter in (1,2,4,5,7,8):
				(self.wp_x,self.wp_y,self.wp_z) = (self.Plant_location[self.iter][0],
												   self.Plant_location[self.iter][1],
												   self.Plant_location[self.iter][2])
				
			elif ((self.iter == 3) and (self.bluecounts >=1)):
				(self.wp_x,self.wp_y,self.wp_z) = (self.Plant_location[self.iter][0],
												   self.Plant_location[self.iter][1], 
												   self.Plant_location[self.iter][2])
				
			elif self.iter == 6 and (self.greencounts < 1):
				self.iter -= 1
				(self.wp_x,self.wp_y,self.wp_z) = (self.Plant_location[self.iter][0],
												   self.Plant_location[self.iter][1], 
												   self.Plant_location[self.iter][2])
	
			elif ((self.iter == 6) and (self.greencounts >= 1)):
				(self.wp_x,self.wp_y,self.wp_z) = (self.Plant_location[self.iter][0],
												   self.Plant_location[self.iter][1], 
												   self.Plant_location[self.iter][2])
					
			elif ((self.iter == 9) and (self.redcounts < 1)):
				
				self.iter-=1
				(self.wp_x,self.wp_y,self.wp_z) = (self.Plant_location[self.iter][0], 
												   self.Plant_location[self.iter][1], 
												   self.Plant_location[self.iter][2])

			elif (self.iter == 9 and (self.redcounts >= 1)):
				(self.wp_x,self.wp_y,self.wp_z) = (self.Plant_location[self.iter][0],
												   self.Plant_location[self.iter][1], 
												   self.Plant_location[self.iter][2])

			elif self.iter > 9:
				(self.wp_x,s elf.wp_y, self.wp_z) = (0.0,0.0,25.0)
				if self.count ==0 :
					
					print('Pollination done!)
					print('STOP')
					self.count+=1			

#publishes the error value which we used for tuning using plotjuggler
	def publish_plot_data(self):
	
        self.yaw_err_data   = self.wp_t-self.drone_t
   		self.alt_err_data   = self.wp_z-self.drone_z
		self.pitch_err_data = self.wp_y-self.drone_y
		self.roll_err_data  = self.wp_x-self.drone_x
   		
   		self.pub.publish(self.alt_err_data)
		self.pub2.publish(self.pitch_err_data)
		self.pub1.publish(self.roll_err_data)
		self.pub3.publish(self.yaw_err_data)

"""
* Function Name:   calc_pid(self)
* Input:	   reference to self,no input
* Output:	   calls the functions self.pid_roll(),self.pid_pitch(),self.pid_throt(),self.pid_yaw()
* Logic:	   none
*
* Example Call:	   called by position_hold(self) as self.calc_pid()


"""   		

	def calc_pid(self):
		self.seconds      = time.time()
		self.current_time = self.seconds - self.last_time
		self.dt           = self.current_time
		if(self.current_time >= self.loop_time):
			self.pid_roll()
			self.pid_pitch()
			self.pid_throt()
			self.pid_yaw()
			
			self.last_time = self.seconds

"""
* Function Name:   pid_yaw(self)
* Input:	   reference to self,no input
* Output:          assigns the corrected value of yaw to self.correct_yaw
* Logic:	   an error is calculated by subtracting its current yaw value from required yaw ,
		   and this error is used to give the corrected value of yaw by applying pid 
*
* Example Call:    self.pid_yaw(), called by calc_pid()


"""
	def pid_yaw(self):
		error = (self.wp_t - self.drone_t)
		self.yaw_iterm += error
		kd = self.kd_yaw
		kp = self.kp_yaw
		ki = self.ki_yaw
		output = kp*error + kd*(error-self.yaw_previous_error)+ki*self.yaw_iterm
		self.yaw_previous_error = error
		self.correct_yaw = output
"""
* Function Name:   pid_roll(self)
* Input:	   reference to self,no input
* Output:	   assigns the corrected value of roll to self.correct_roll
* Logic:	   an error is calculated by subtracting its current roll value from required roll ,
		   and this error is used to give the corrected value of roll by applying pid 

* Example Call:	   self.pid_roll(), called by calc_pid()


"""

	def pid_roll(self):

		#Compute Roll PID here
		error = (self.wp_x-self.drone_x)
		
		
	
		self.roll_iterm+=error
		kd = self.kd_roll
		kp = self.kp_roll
		ki = self.ki_roll
		output = kp*error + kd*(error-self.roll_previous_error)/self.dt +ki*self.roll_iterm*self.dt
		self.roll_previous_error = error
		self.correct_roll = output

"""
* Function Name:   pid_pitch(self)  
* Input:	   reference to self,no input
* Output:	   assigns the corrected value of pitch to self.correct_pitch
* Logic:	   an error is calculated by subtracting its current pitch value from required pitch ,
		   and this error is used to give the corrected value of pitch by applying pid 

* Example Call:	   self.pid_pitch(), called by calc_pid()


"""
	def pid_pitch(self):

		#Compute Pitch PID here
		error = (self.wp_y-self.drone_y)
		
		
		self.pitch_iterm+=error
		kd = self.kd_pitch
		kp = self.kp_pitch
		ki = self.ki_pitch
		output = kp*error + kd*(error-self.pitch_previous_error)/self.dt +ki*self.pitch_iterm*self.dt
		self.pitch_previous_error = error
	
		self.correct_pitch = output

"""
* Function Name:   pid_throt(self) 
* Input:	   reference to self,no input
* Output:	   assigns the corrected value of pitch to self.correct_throt
* Logic:	   an error is calculated by subtracting its current throttle value from required throttle ,
		   and this error is used to give the corrected value of pitch by applying pid 

* Example Call:	   self.pid_throt(), called by calc_pid() 


"""
	def pid_throt(self):

		#Compute Throttle PID here
		error = (self.wp_z-self.drone_z)
		
		
		self.throt_iterm+=error
		kd = self.kd_throt
		kp = self.kp_throt
		ki = self.ki_throt
		output = kp*error + kd*(error-self.throt_previous_error)/self.dt +self.throt_iterm*ki*self.dt
		self.throt_previous_error = error
		self.correct_throt = output


"""
* Function Name:  limit(self, input_value, max_value, min_value): 
* Input:	  integer value ,input is sent by position_hold(self) function
* Output:	  returns the value of max_value and min_value
* Logic:	  compares input value ,if it greater than max value it returns max_value if it is less than min_value returns min_value,else returns input_value
*
* Example Call:	  self.limit(self,integer,integer,integer),called by position_hold(self) function


"""	

	def limit(self, input_value, max_value, min_value):

		#Use this function to limit the maximum and minimum values you send to your drone

		if input_value > max_value:
			return max_value
		if input_value < min_value:
			return min_value
		else:
			return input_value

			

"""
* Function Name:   get_pose(self,pose)
* Input:	   float,it gets the value obtained by subscribing to /whycon/poses topic 
* Output:	   updates the value of currnt x,y and z coordinate
* Logic:	   none
*
* Example Call:	  called by a whycon_pose subscriber as self.get_pose


"""		
		
	def get_pose(self,pose):

		#This is the subscriber function to get the whycon poses
		#The x, y and z values are stored within the drone_x, drone_y and the drone_z variables
		
		self.drone_x = pose.poses[0].position.x
		self.drone_y = pose.poses[0].position.y
		self.drone_z = pose.poses[0].position.z

"""
* Function Name:   get_t(self,Float64)
* Input:	   it gets the value obtained through /whycon/poses topic
* Output:	   updates the value of current drone angle
* Logic:	   none
*
* Example Call:	   galled by yaw subscriber as self.get_t


"""

	def get_t(self,Float64):
		self.drone_t=Float64.data
"""
* Function Name:   get_red(self,Int32)
* Input:	   it gets the contour value for red plant
* Output:	   updates the value of self.redcounts
* Logic:	   none
*
* Example Call:	   called by red pollinated flowers count subscriber as self.get_red


"""
	def get_red(self,Int32):
		self.redcounts=Int32.data
"""
* Function Name:   get_green(self,Float64)
* Input:	   it gets the contour value for green plant
* Output:	   updates the value of self.greencounts
* Logic:	   none
*
* Example Call:	   called by green pollinated flowers count subscriber as self.get_green


"""
	def get_green(self,Int32):
		self.greencounts=Int32.data
"""
* Function Name:   get_blue(self,Float64)
* Input:	   it gets the contour value for blue plant
* Output:	   updates the value of self.bluecounts
* Logic:	   none
*
* Example Call:	   called by blue pollinated flowers count subscriber as self.get_blue

"""
	def get_blue(self,Int32):
		self.bluecounts=Int32.data
	
		
if __name__ == '__main__':
	while not rospy.is_shutdown():
		temp = DroneFly()
		temp.position_hold()
		rospy.spin()
