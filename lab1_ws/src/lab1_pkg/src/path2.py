import numpy as np
import math
from utils import *
import time

"""
Starter script for lab1. 
Author: Chris Correa
"""

# IMPORTANT: the init methods in this file may require extra parameters not 
# included in the starter code.  

class MotionPath:
	def target_position(self, time):
		raise NotImplementedError

	def target_velocity(self, time):
		raise NotImplementedError

	def target_acceleration(self, time):
		raise NotImplementedError

	def is_finished(self, time):
		raise NotImplementedError

class LinearPath(MotionPath):
	def __init__(self, ar_tag, start_pos, start_time = 0):
		self.start_pos = start_pos
		self.final_pos = ar_tag
		self.targ_velocity = .2*(self.final_pos - self.start_pos)
		self.targ_velocity[2] = 0
		self.start_time = start_time

	def target_position(self, time):
		targ_pos = self.start_pos  + (time-self.start_time)*self.targ_velocity
		return targ_pos

	def target_velocity(self, time):
		return self.targ_velocity

	def target_acceleration(self, time):
		return np.zeros((6,1))

	def is_finished(self, position, t=None):
		#print(np.linalg.norm(self.final_pos[:2] - position[:2]))
		if(np.linalg.norm(self.final_pos[:2] - position[:2]) < 0.05):
			return True
		return False



class CircularPath(MotionPath):
	def __init__(self, ar_tag, start_pos):
		self.r = np.linalg.norm(ar_tag[:2] - start_pos[:2])
		self.z = start_pos[2]
		self.origin = ar_tag
		self.init_theta = np.arcsin((start_pos[1]-ar_tag[1])/self.r)
		print("init theta is: ", self.init_theta)
		self.theta_inc = np.pi/20
	
	def target_position(self, time):
		print(np.linalg.norm([self.r*np.cos(self.theta_inc*time),self.r*np.sin(self.theta_inc*time)]))
		return np.array([self.r*np.cos(self.init_theta + self.theta_inc*time) + self.origin[0],self.r*np.sin(self.init_theta + self.theta_inc*time)+ self.origin[1], self.z])

	def target_velocity(self, time):
		return np.array([-self.r*np.sin(self.init_theta + self.theta_inc*time)*(self.theta_inc),self.r*np.cos(self.init_theta + self.theta_inc*time)*(self.theta_inc), 0])

	def target_acceleration(self, time):
		return np.array([-self.r*np.cos(self.init_theta + self.theta_inc*time)*(self.theta_inc)**2,-self.r*np.sin(self.init_theta + self.theta_inc*time)*(self.theta_inc)**2, 0,0,0,0])

	def is_finished(self, position, t):
		# print(self.end_pos)
		# print(np.linalg.norm(target_position - np.array(position[:2])), self.r)
		# if(np.linalg.norm(self.end_pos - np.array(position[:2])) < 0.05):
		# 	return True
		# return False
		if t < 40:
			return False
		return True

	# def target_acceleration(self, time):

# # You can implement multiple paths a couple ways.  The way I chose when I took
# # the class was to create several different paths and pass those into the 
# # MultiplePaths object, which would determine when to go onto the next path.

class MultiplePaths(MotionPath):
	def __init__(self, start_pos, end_points):
		self.count = 0
		self.end_points = end_points
		self.current_start = start_pos
		self.current_path = LinearPath(self.end_points[self.count], self.current_start)

	def target_position(self, time):
		return self.current_path.target_position(time)

	def target_velocity(self, time):
		return self.current_path.target_velocity(time)

	def target_acceleration(self, time):
		return self.current_path.target_acceleration(time)

	def is_finished(self, position, t=None):
		print(self.count)
		if self.count == 5:
			return True
		else:
			if self.current_path.is_finished(position):
				self.count += 1
				self.current_start = position
				self.current_path = LinearPath(self.end_points[self.count % 4], self.current_start, t)
				time.sleep(1)
			return False


class VisualServoPaths(MotionPath):
	def __init__(self, start_pos, end_point, limb):
		# self.count = 0
		self.end_point = end_point
		self.current_start = start_pos
		self.current_path = LinearPath(self.end_point, self.current_start)
		self.limb = limb

	def target_position(self, time):
		return self.current_path.target_position(time)

	def target_velocity(self, time):
		return self.current_path.target_velocity(time)

	def target_acceleration(self, time):
		return self.current_path.target_acceleration(time)

	def is_finished(self, position, t=None, servo = False, lookup_tag = None):
		# print(self.count)
		# if self.count == 5:
		# 	return True
		# else:
		if self.current_path.is_finished(position):
			print("finished")
			return True
			# self.count += 1
		else:
			if servo:
				tag_pos = np.array(lookup_tag())
				if tag_pos is None:
					print("tag not found")
				elif  np.linalg.norm(self.current_path.final_pos - tag_pos) > 0.1:
					self.current_start = self.limb.endpoint_pose()["position"]
					self.current_path = LinearPath(tag_pos, self.current_start, t)
			# time.sleep(1)
			return False