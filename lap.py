#!/usr/bin/env python2
import numpy as np
import rospy
import time
from std_msgs.msg import String, Float64
from sensor_msgs.msg import *
from mavros_msgs.srv import *
from mavros_msgs.msg import *
from geometry_msgs.msg import *
import math
from time import sleep
from tf.transformations import euler_from_quaternion
ARM_RAD=1
N_laps = 2



class stateMoniter:
	def __init__(self):
		self.state = State()
		# Instantiate a setpoints message
		
	def stateCb(self, msg):
		# Callback function for topic /mavros/state
		self.state = msg

class wpMissionCnt:

	def __init__(self):
		self.wp =Waypoint()
		
	def setWaypoints(self,frame,command,is_current,autocontinue,param1,param2,param3,param4,x_lat,y_long,z_alt):
		self.wp.frame =frame #  FRAME_GLOBAL_REL_ALT = 3 for more visit http://docs.ros.og/api/mavros_msgs/html/msg/Waypoint.html
		self.wp.command = command #VTOL TAKEOFF = 84,NAV_WAYPOINT = 16, TAKE_OFF=22 for checking out other parameters go to https://github.com/mavlink/mavros/blob/master/mavros_msgs/msg/CommandCode.msg'''
		self.wp.is_current= is_current
		self.wp.autocontinue = autocontinue # enable taking and following upcoming waypoints automatically 
		self.wp.param1=param1 # To know more about these params, visit https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_WAYPOINT
		self.wp.param2=param2
		self.wp.param3=param3
		self.wp.param4=param4
		self.wp.x_lat= x_lat 
		self.wp.y_long=y_long
		self.wp.z_alt= z_alt #relative altitude.

		return self.wp

class FLIGHT_CONTROLLER:

	def __init__(self):
		self.pt = Point()
		self.orient = Quaternion()
		self.angles = Point()
		self.gps = Point()
		stateMt = stateMoniter()
		self.transformation_matrix = np.array([[0, -1, 0], [1,0,0], [0,0,1]])
		#NODE
		rospy.init_node('iris_drone', anonymous = True)

		#SUBSCRIBERS
		self.get_pose_subscriber = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.get_pose)
		# self.get_linear_vel=rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self.get_vel,)
		# self.get_imu_data=rospy.Subscriber('/mavros/imu/data',Imu,self.get_euler_angles)
		self.get_gps_location = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.get_gps)
		self.state_subsciber = rospy.Subscriber('/mavros/state',State, stateMt.stateCb)
		rospy.Subscriber("/mavros/local_position/pose",PoseStamped, self.get_yaw)

		#PUBLISHERS
		self.publish_pose = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped,queue_size=10)
		self.publish_attitude_thrust=rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget,queue_size=0)

		#SERVICES
		self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
		self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
		self.land_service = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
		self.flight_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
		self.waypoint_push = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)
		self.waypoint_curr = rospy.ServiceProxy('/mavros/mission/set_current', WaypointSetCurrent)

		rospy.loginfo('INIT')

		self.pt.x = 2
		self.pt.y = 2
		self.pt.z = 2

	#MODE SETUP

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

	def wpPush(self,wps):
		# Call /mavros/mission/push to push the waypoints
		# and print fail message on failure
		rospy.wait_for_service('/mavros/mission/push')
		try:
			self.waypoint_push(0, wps)
			rospy.loginfo("Waypoint pushed")

		except:
			print ("Service waypoint push call failed")



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

	def get_gps(self, data):
		self.gps.x = data.latitude
		self.gps.y = data.longitude
		self.gps.z = data.altitude


	def get_pose(self, location_data):
		self.pt.x = location_data.pose.position.x
		self.pt.y = location_data.pose.position.y
		self.pt.z = location_data.pose.position.z

		# orientation in space  
		self.orient.x = location_data.pose.orientation.x
		self.orient.y = location_data.pose.orientation.y
		self.orient.z = location_data.pose.orientation.z
		self.orient.w = location_data.pose.orientation.w

	def within_rad(self):
		if (((self.pt.x)**2 + (self.pt.y)**2 + (self.pt.z)**2) < (ARM_RAD)**2):
			return True
		print((self.pt.x)**2 + (self.pt.y)**2 + (self.pt.z)**2)
		return False

# this function rotates the coordinates considering the spawn point and direction as the origin
	def rotate(self, point):
		angle = self.angles.z - math.pi/2
		px, py = point[0], point[1]
		nx =  math.cos(angle) * px  - math.sin(angle) * py 
		ny =  math.sin(angle) * px  + math.cos(angle) * py 
		return [nx, ny, point[2]]
		
	def get_eulers(self,q):
		eulers = euler_from_quaternion(q)
		return eulers[2]

	def get_yaw(self, data):
		q = []
		q.append(data.pose.orientation.x)
		q.append(data.pose.orientation.y)
		q.append(data.pose.orientation.z)
		q.append(data.pose.orientation.w)
		angle  = self.get_eulers(q)
		self.angles.x = 0.0
		self.angles.y = 0.0
		self.angles.z = angle
		# print(angle)

		
# this function corrects the frame offset converting the ardupilot frame (x-right, y-front) to 
# gazebo local frame (x-front, y-left)
	def corrected_pose(self, current_pos):
		current_pos = np.array(current_pos)
		new_pos = np.matmul(self.transformation_matrix,current_pos)
		return list(new_pos)


	#PUBLISHERS
	def gotopose(self,x,y,z):
		rate = rospy.Rate(20)
		sp = PoseStamped()
		sp.pose.position.x = x
		sp.pose.position.y = y
		sp.pose.position.z = z

		ix = self.orient.x
		iy = self.orient.y
		iz = self.orient.z
		iw = self.orient.w

		sp.pose.orientation.x = ix
		sp.pose.orientation.y = iy
		sp.pose.orientation.z = iz
		sp.pose.orientation.w = iw


		dist = np.sqrt(((self.pt.x-x)**2) + ((self.pt.y-y)**2) + ((self.pt.z-z)**2))
		i = 0
		while(i < 40):
			self.publish_pose.publish(sp)
			#dist = np.sqrt(((self.pt.x-x)**2) + ((self.pt.y-y)**2) + ((self.pt.z-z)**2))
			i = i+1
			rate.sleep()
		#print('Reached ',x,y,z)



# helper functions for xy to latlon conversions
def mdeglon(lat0):
	lat0rad = math.radians(lat0)
	return (111415.13 * math.cos(lat0rad)- 94.55 * math.cos(3.0*lat0rad)- 0.12 * math.cos(5.0*lat0rad) )

def mdeglat(lat0):
	lat0rad = math.radians(lat0)
	return (111132.09 - 566.05 * math.cos(2.0*lat0rad)+ 1.20 * math.cos(4.0*lat0rad)- 0.002 * math.cos(6.0*lat0rad) )

# xy to latlon conversions

def xy2latlon(local, origin):
	lon = local[0]/mdeglon(origin[0]) + origin[1]
	lat = local[1]/mdeglat(origin[1]) + origin[0]
	alt = local[2] + origin[2]
	gl = np.array([lat, lon, alt])
	return gl

def latlon2xy(coordinates, origin):
	x = (coordinates[1]- origin[1]) + mdeglon(origin[0])
	y = (coordinates[0] - origin[0]) + mdeglat(origin[0])
	return x,y
	

if __name__ == '__main__':

	mav = FLIGHT_CONTROLLER()
	stateMt = stateMoniter()

	rate= rospy.Rate(20.0)
	time.sleep(3)
	print(mav.within_rad())
	if (True):		
		# defining waypoints
		wayp_11 = wpMissionCnt()
		wayp_12 = wpMissionCnt()
		wayp_13 = wpMissionCnt()
		wayp_21 = wpMissionCnt()
		wayp_22 = wpMissionCnt()
		wayp_23 = wpMissionCnt()
		wayp_land = wpMissionCnt()

		# defining the origin as the gps coordinates of the spawn point
		origin = [mav.gps.x, mav.gps.y, 0]
		
		# coordinates given in the gazebo frame (x-front, y- left)
		wayp11 = [50,0,5]
		wayp12 = [55,5,5]
		wayp13 = [50,10,5]
		wayp21 = [0,10,5]
		wayp22 = [-5,5,5]
		wayp23 = [0,0,5]
		waypoints = [wayp11,wayp12,wayp13,wayp21,wayp22,wayp23]
		transformed_waypoints = []
		# the coordinates are received in ardupilot frame (x- right, y- front)

		# converting the ardupilot frame coordinates to gazebo frame coordinates
		for waypoint in waypoints:
			waypoint = mav.corrected_pose(waypoint)
			waypoint = mav.rotate(waypoint)
			waypoint = xy2latlon(waypoint, origin)
			transformed_waypoints.append(waypoint)


		wps = []
		w_1 = wayp_11.setWaypoints(3,16,False,True,0.0,0.0,0.0,float('nan'),transformed_waypoints[0][0],transformed_waypoints[0][1],transformed_waypoints[0][2])
		w_2 = wayp_12.setWaypoints(3,16,False,True,0.0,0.0,0.0,float('nan'),transformed_waypoints[1][0],transformed_waypoints[1][1],transformed_waypoints[1][2])
		w_3 = wayp_13.setWaypoints(3,16,False,True,0.0,0.0,0.0,float('nan'),transformed_waypoints[2][0],transformed_waypoints[2][1],transformed_waypoints[2][2])
		w_4 = wayp_21.setWaypoints(3,16,False,True,0.0,0.0,0.0,float('nan'),transformed_waypoints[3][0],transformed_waypoints[3][1],transformed_waypoints[3][2])
		w_5 = wayp_22.setWaypoints(3,16,False,True,0.0,0.0,0.0,float('nan'),transformed_waypoints[4][0],transformed_waypoints[4][1],transformed_waypoints[4][2])
		w_6 = wayp_23.setWaypoints(3,16,False,True,0.0,0.0,0.0,float('nan'),transformed_waypoints[5][0],transformed_waypoints[5][1],transformed_waypoints[5][2])
		w_land = wayp_land.setWaypoints(3,21,False,True,0.0,0.0,0.0,float('nan'),transformed_waypoints[5][0],transformed_waypoints[5][1],transformed_waypoints[5][2])
		
		wps.append(w_1)

		for i in range(N_laps):
			wps.append(w_1)
			wps.append(w_2)
			wps.append(w_3)
			wps.append(w_4)
			wps.append(w_5)
			if i == (N_laps-1):
				wps.append(w_land)
			else:
				wps.append(w_6)
		mav.wpPush(wps)
		print(wps)
	
		# 16 -> NAVIGATE
		# 21 -> LAND
		# 22 -> TAKEOFF

		mav.set_mode('STABILIZE')
		mav.toggle_arm(1)
		time.sleep(3)
		mav.set_Guided_mode()
		mav.takeoff(5)
		time.sleep(7)
		mav.set_Guided_mode()
		mav.set_mode("AUTO")	
		mav.toggle_arm(0)

