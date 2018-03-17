# imports may be necessary
import numpy as np
from geometry_msgs.msg import Twist, Vector3

class Controller():
	def __init__(self, path, k, target_speed, obstacle, obstacle_center, obstacle_radius):
		self.path = path
		self.target_speed = target_speed
		self.obstacle = obstacle
		self.obstacle_center = obstacle_center
		self.obstacle_radius = obstacle_radius
		self.K1 = k[0]
		self.K2 = k[1]
		self.K3 = k[2]

	def step_path(self, current_state, s):
		"""
		Takes the current state and final state and returns the twist command to reach the target state
		according to the path variable

		Parameters
		----------
		current_state: :obj:`numpy.ndarray`
			twist representing the current state of the turtlebot.  see utils.py
		s: float
			the path length the turtlebot should have travelled so far

		Returns
		-------
		:obj:`geometry_msgs.msg.Twist`
			Twist message to be sent to the turtlebot

		"""
		toRet = Twist()

		x_d,y_d,theta_d = self.path.target_state(s)
		x,y,theta = current_state
		# v_d, w_d = self.path.target_velocity(s).linear, self.path.target_velocity(s).angular
		v_d, w_d = self.path.target_velocity(s)
		sign = self.path.going_back
		# sign = 1.0

		# if x_d > 0:
		# 	e = x_d - x
		# else:
		# 	e = x - x_d
		e = x_d - x

		if abs(theta_d) > np.pi/2:
			e = -e


		e_theta = (theta_d - theta)
		e_theta = (e_theta+np.pi)%(2*np.pi) - np.pi

		# if e_theta > np.pi:
		# 	e_theta -= 2*np.pi

		# if e_theta < -np.pi:
		# 	e_theta += 2*np.pi

		# print e_theta

		# toRet.linear = Vector3(*(np.cos(theta_d - theta)*self.target_speed + self.K1*(x_d-x)).tolist())
		# toRet.angular = Vector3(*(w_d - self.K2*self.target_speed*np.sinc(theta_d + theta)*(y_d-y) - self.K3*(theta_d - theta)).tolist())
		toRet.linear.x = sign*np.cos(theta_d - theta)*self.target_speed + self.K1*e
		toRet.angular.z = w_d[2] - sign*self.K2*self.target_speed*np.sinc(e_theta)*(y_d-y) + self.K3*(e_theta)

		# linear = np.cos(theta_d - theta)*v_d - self.K1*(x_d-x)
		# angular = w_d + self.K2*v_d*np.sinc(theta_d - theta)*(y_d-y) - self.K3*(theta_d - theta)

		# toRet.linear.x = linear[0]
		# toRet.linear.y = linear[1]
		# toRet.linear.z = linear[2]

		# toRet.angular.x = angular[0]
		# toRet.angular.y = angular[1]
		# toRet.angular.z = angular[2]

		return toRet