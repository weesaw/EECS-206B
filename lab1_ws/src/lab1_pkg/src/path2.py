import numpy as np
import math
from utils import *

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
	def __init__(self, ar_tag, start_pos):
		self.start_pos = start_pos
		self.final_pos = ar_tag
		self.targ_velocity = .7*(self.final_pos - self.start_pos)
		self.targ_velocity[2] = 0

	def target_position(self, time):
		targ_pos = self.start_pos  + time*self.targ_velocity
		return targ_pos

	def target_velocity(self, time):
		return self.targ_velocity

	def target_acceleration(self, time):
		return np.zeros((6,1))

	def is_finished(self, position, t=None):
		print(np.linalg.norm(self.final_pos[:2] - position[:2]))
		if(np.linalg.norm(self.final_pos[:2] - position[:2]) < 0.04):
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

# class MultiplePaths(MotionPath):
#   def __init__(self, paths):
#         return

#   def get_current_path(self, time):
#         return