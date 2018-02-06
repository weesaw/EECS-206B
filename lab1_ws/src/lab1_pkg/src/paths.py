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

class LinearPathWorkspaceControl(MotionPath):
	def __init__(self, ar_tag):
        self.targ_position = ar_tag
        self.targ_velocity = 0.3
        self.targ_acceleration = 0

    def target_position(self, time):
        return self.targ_position

    def target_velocity(self, time, position):
        return self.targ_velocity

    def target_acceleration(self, time):
        return self.targ_acceleration

    def is_finished(self, time, position): 
        if(np.linalg.norm(self.target_position(time) - position) < 0.2):
            return True
        return False

class LinearPathJointspaceControl(MotionPath):
    def __init__(self, ar_tag):
        

class CircularPath(MotionPath):
	def __init__():

# You can implement multiple paths a couple ways.  The way I chose when I took
# the class was to create several different paths and pass those into the 
# MultiplePaths object, which would determine when to go onto the next path.

class MultiplePaths(MotionPath):
	def __init__(self, paths):

	def get_current_path(self, time):