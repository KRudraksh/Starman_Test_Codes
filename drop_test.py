#!/usr/bin/env python2
import numpy as np
import rospy
import time
from std_msgs.msg import String, Float64
from sensor_msgs.msg import NavSatFix, Image,Imu, Range
from mavros_msgs.srv import CommandTOL, SetMode, CommandBool
from mavros_msgs.msg import AttitudeTarget, OverrideRCIn
from geometry_msgs.msg import PoseStamped, Pose, Point, Twist, TwistStamped
import math
from time import sleep
ARM_RAD=1

class FLIGHT_CONTROLLER:

	def __init__(self):
		self.pt = Point()
		self.vel = Point()
		self.alt = Range()
		#NODE
		rospy.init_node('new', anonymous = True)

		#SUBSCRIBERS
		self.get_pose_subscriber = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.get_pose)
		self.get_linear_vel=rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped,self.get_velocity)
		self.get_imu=rospy.Subscriber("/mavros/imu/data", Imu, self.imu_call)
		# self.get_imu_data=rospy.Subscriber('/mavros/imu/data',Imu,self.get_euler_angles)
		rospy.Subscriber('/mavros/rangefinder/rangefinder', Range, self.rangefinder )

		#PUBLISHERS
		self.publish_pose = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped,queue_size=10)
		self.publish_attitude_thrust=rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget,queue_size=0)
		self.publish_overriderc=rospy.Publisher('/mavros/rc/override', OverrideRCIn,  queue_size=10)

		#SERVICES
		self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
		self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
		self.land_service = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
		self.flight_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)

		rospy.loginfo('INIT')

	#MODE SETUP

	def get_velocity(self,data):
		self.vel.x = data.twist.linear.x
		self.vel.y = data.twist.linear.y
		self.vel.z = data.twist.linear.z


	def toggle_arm(self, arm_bool):
		rospy.wait_for_service('/mavros/cmd/arming')
		try:
			self.arm_service(arm_bool)
		
		except rospy.ServiceException as e:
			rospy.loginfo("Service call failed: " %e)

	def takeoff(self, t_alt):
		# self.gps_subscriber

		# t_lat = self.gps_lat
		# t_long = self.gps_long

		rospy.wait_for_service('/mavros/cmd/takeoff')
		try:
			self.takeoff_service(0,0,0,0,t_alt)
			rospy.loginfo('TAKEOFF')
		except rospy.ServiceException as e:
			rospy.loginfo("Service call failed: " %e)
	
	
	def land(self, l_alt):

		# self.gps_subscriber

		# l_lat = self.gps_lat
		# l_long = self.gps_long

		rospy.wait_for_service('/mavros/cmd/land')
		try:
			self.land_service(0.0, 0.0, 0, 0, l_alt)
			rospy.loginfo("LANDING")

		except rospy.ServiceException as e:
			rospy.loginfo("Service call failed: " %e)


	def set_mode(self,md):

			rospy.wait_for_service('/mavros/set_mode')
			try:
				self.flight_mode_service(0, md)
				rospy.loginfo("Mode changed")
				
			except rospy.ServiceException as e:
				rospy.loginfo("Mode could not be set: " %e)

	def set_Guided_mode(self):
		
		rate=rospy.Rate(20)
		#print('OFF')
		PS = PoseStamped()

		PS.pose.position.x = 0
		PS.pose.position.y = 0
		PS.pose.position.z = 0
		
		for i in range(10):
			self.publish_pose.publish(PS)
			
			rate.sleep()
		print('done')
		self.set_mode("GUIDED")

	def set_Altitude_Hold_mode(self):

		rate=rospy.Rate(20)
		#print('OFF')
		PS = PoseStamped()

		PS.pose.position.x = 0
		PS.pose.position.y = 0
		PS.pose.position.z = 0
		
		for i in range(10):
			self.publish_pose.publish(PS)
			
			rate.sleep()
		print('done')
		self.set_mode("ALT_HOLD")	

	#CALLBACKS

	# def gps_callback(self, data):
	# 	self.gps_lat = data.latitude
	# 	self.gps_long = data.longitude


	def get_pose(self, location_data):
		self.pt.x = location_data.pose.position.x
		self.pt.y = location_data.pose.position.y
		self.pt.z = location_data.pose.position.z

	def imu_call(self, imu_val):
		self.z2 = imu_val.linear_acceleration.z


	# def get_vel(self,vel_data):
	# 	self.x_vel=	vel_data.twist.linear.x
	# 	self.y_vel=	vel_data.twist.linear.y
	# 	self.z_vel=	vel_data.twist.linear.z
		
	def within_rad(self):
		if (((self.pt.x)**2 + (self.pt.y)**2 + (self.pt.z)**2) < (ARM_RAD)**2):
			return True
		print((self.pt.x)**2 + (self.pt.y)**2 + (self.pt.z)**2)
		return False


	def set_throttle(self,t):
		print("Setting Thottle ")
		rate = rospy.Rate(20)
		rc = OverrideRCIn()
		rc.channels[2]=t
		i = 0
		for i in range (100):
			self.publish_overriderc.publish(rc)
			i=i+1

	def rangefinder(self, data):
		self.alt.range = data.range




	#PUBLISHERS
	def gotopose(self,x,y,z):
		rate = rospy.Rate(20)
		sp = PoseStamped()
		sp.pose.position.x = x
		sp.pose.position.y = y
		sp.pose.position.z = z
		sp.pose.orientation.x = 0.0
		sp.pose.orientation.y = 0.0
		sp.pose.orientation.z = 0.0
		sp.pose.orientation.w = 1.0
		dist = np.sqrt(((self.pt.x-x)**2) + ((self.pt.y-y)**2) + ((self.pt.z-z)**2))
		while(dist > 0.2):
			self.publish_pose.publish(sp)
			dist = np.sqrt(((self.pt.x-x)**2) + ((self.pt.y-y)**2) + ((self.pt.z-z)**2))
			rate.sleep()
		#print('Reached ',x,y,z)


	# def set_pose(self):

	# 	update_rate = rospy.Rate(20)
	# 	PS = PoseStamped()

	# 	PS.pose.position.x = self.set_x
	# 	PS.pose.position.y = self.set_y
	# 	PS.pose.position.z = self.set_z

	# 	PS.pose.orientation.x = 0
	# 	PS.pose.orientation.y = 0
	# 	PS.pose.orientation.z = 0.707
	# 	PS.pose.orientation.w = 0.707

	# 	distance =math.sqrt((self.set_x - self.curr_x)**2 + (self.set_y - self.curr_y)**2 + (self.set_z - self.curr_z)**2)

	# 	while (distance > self.delta): #and (abs(self.set_z - self.curr_z) > self.delta_z):

	# 		self.publish_pose.publish(PS)
	# 		self.get_pose_subscriber
	# 		distance =math.sqrt((self.set_x - self.curr_x)**2 + (self.set_y - self.curr_y)**2 + (self.set_z - self.curr_z)**2)
	# 		self.rgb_flag = 0
	# 		self.depth_flag = 0
	# 		update_rate.sleep()

	# 	self.waypoint_number = self.waypoint_number + 1
	# 	#self.depth_flag = 1
	# 	#rospy.loginfo('WAYPOINT REACHED: ' + str(self.waypoint_number))
		


if __name__ == '__main__':

	mav = FLIGHT_CONTROLLER()
	rate = rospy.Rate(50)
	time.sleep(3)
	print(mav.within_rad())

	if (mav.within_rad()):
		mav.set_mode('STABILIZE')
		mav.toggle_arm(1)

		rc_t = OverrideRCIn()
		print('start drop')
		mav.set_mode('ALT_HOLD')
		# time.sleep(10)
		zo = mav.alt.range
		zin = mav.alt.range
		print(zo)
		drop = False
		rc_t.channels[2]=1400
		mav.publish_overriderc.publish(rc_t)
		while(True):
			if(0<mav.z2<1):
				rc_t.channels[2]=1400
				mav.publish_overriderc.publish(rc_t)
				drop = True

			if(zin-zo < -1 and drop):
				rc_t.channels[2]=1500
				mav.publish_overriderc.publish(rc_t)

				break
			zin = mav.alt.range
	print('Drop Complete')
	mav.set_Guided_mode()
	mav.gotopose(2,2,2)
	time.sleep(5)
	mav.land(5)
	mav.toggle_arm(0)



