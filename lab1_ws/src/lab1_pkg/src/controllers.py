import rospy
import numpy as np
from utils import *
from geometry_msgs.msg import PoseStamped
import time
"""
Starter script for lab1. 
Author: Chris Correa
"""
class Controller:
	joint_order = ["left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"]

	def step_path(self, path, t):
		raise NotImplementedError

	def finished(self, path, t):
		raise NotImplementedError

	def execute_path(self, path, finished, timeout=None, log=False):
		start_t = rospy.Time.now()
		times = list()
		actual_positions = list()
		actual_velocities = list()
		target_positions = list()
		target_velocities = list()
		r = rospy.Rate(200)
		while True:
			t = (rospy.Time.now() - start_t).to_sec()
			if timeout is not None and t >= timeout:
				return False
			self.step_path(path, t)
			if log:
				times.append(t)
				# actual_positions.append(self.current_joint_pos)
				# actual_velocities.append(self.current_joint_vel)
				actual_positions.append(self.limb.joint_angles)
				actual_velocities.append(self.limb.joint_velocities)
				target_positions.append(path.target_position(t))
				target_velocities.append(path.target_velocity(t))
			# print(path)
			check_pos = self.limb.endpoint_pose()["position"]
			if finished is not None and finished(check_pos):
				break
			r.sleep()

		if log:
			import matplotlib.pyplot as plt

			np_actual_positions = np.zeros((len(times), 3))
			np_actual_velocities = np.zeros((len(times), 3))
			for i in range(len(times)):
				# print actual_positions[i]
				actual_positions_dict = dict((joint, actual_positions[i][j]) for j, joint in enumerate(self.limb.joint_names()))
				print "dictionary version", actual_positions_dict
				np_actual_positions[i] = self.kin.forward_position_kinematics(joint_values=actual_positions_dict)[:3]
				np_actual_velocities[i] = self.kin.jacobian(joint_values=actual_positions_dict)[:3].dot(actual_velocities[i])
			target_positions = np.array(target_positions)
			target_velocities = np.array(target_velocities)
			plt.figure()
			# print len(times), actual_positions.shape()
			plt.subplot(3,2,1)
			plt.plot(times, np_actual_positions[:,0], label='Actual')
			plt.plot(times, target_positions[:,0], label='Desired')
			plt.xlabel("Time (t)")
			plt.ylabel("X Position Error")

			plt.subplot(3,2,2)
			plt.plot(times, np_actual_velocities[:,0], label='Actual')
			plt.plot(times, target_velocities[:,0], label='Desired')
			plt.xlabel("Time (t)")
			plt.ylabel("X Velocity Error")
			
			plt.subplot(3,2,3)
			plt.plot(times, np_actual_positions[:,1], label='Actual')
			plt.plot(times, target_positions[:,1], label='Desired')
			plt.xlabel("time (t)")
			plt.ylabel("Y Position Error")

			plt.subplot(3,2,4)
			plt.plot(times, np_actual_velocities[:,1], label='Actual')
			plt.plot(times, target_velocities[:,1], label='Desired')
			plt.xlabel("Time (t)")
			plt.ylabel("Y Velocity Error")
			
			plt.subplot(3,2,5)
			plt.plot(times, np_actual_positions[:,2], label='Actual')
			plt.plot(times, target_positions[:,2], label='Desired')
			plt.xlabel("time (t)")
			plt.ylabel("Z Position Error")

			plt.subplot(3,2,6)
			plt.plot(times, np_actual_velocities[:,2], label='Actual')
			plt.plot(times, target_velocities[:,2], label='Desired')
			plt.xlabel("Time (t)")
			plt.ylabel("Z Velocity Error")

			plt.show()

		return True

class PDWorkspaceVelocityController(Controller):
	def __init__(self, limb, kin, Kp, Kv):
		self.limb = limb
		self.kin = kin
		self.Kp = Kp
		self.Kv = Kv

	def step_path(self, path, t):
		error = self.limb.endpoint_pose()['position'] - path.target_position(t)
		error_dot = self.limb.endpoint_velocity()['linear'] - path.target_velocity(t)
		new_vel = self.Kp*error + self.Kv*error_dot

		# print(error, error_dot)

		new_vel = np.append(new_vel, np.zeros((3,1)))
		joint_vels = np.dot(self.kin.jacobian_pseudo_inverse(), new_vel)

		print(joint_vels)

		dict_ = {}
		keys = self.limb.joint_names()
		for i in range(len(keys)):
			dict_[keys[i]] = joint_vels.item(i)

		self.limb.set_joint_velocities(dict_)

		time.sleep(.5)



class PDJointVelocityController(Controller):
	def __init__(self, limb, kin, Kp, Kv):
		self.limb = limb
		self.kin = kin
		self.Kp = Kp
		self.Kv = Kv

	def step_path(self, path, t):
		# dict_vels = self.limb.joint_velocities()
		dict_pos = self.limb.joint_angles()
		# current_joint_vel = np.array([dict_vels[k] for k in ["left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"]])
		current_joint_pos = np.array([dict_pos[k] for k in Controller.joint_order])
	
		joint_error = current_joint_pos - self.kin.inverse_kinematics(path.target_position(t))
		
		workspace_error = self.limb.endpoint_velocity()["linear"] - path.target_velocity(t)
		joint_error_dot = np.dot(self.kin.jacobian_pseudo_inverse(), np.append(workspace_error,np.zeros((3,1))))

		new_vel = self.Kp*joint_error + self.Kv*joint_error_dot

		# print(new_vel)

		dict_ = {}
		for i in range(len(Controller.joint_order)):
			dict_[Controller.joint_order[i]] = new_vel.item(i)

		# print(new_vel)

		self.limb.set_joint_velocities(dict_)

class PDJointTorqueController(Controller):
	def __init__(self, limb, kin, Kp, Kv):
		self.limb = limb
		self.kin = kin
		self.Kp = Kp
		self.Kv = Kv

	def step_path(self, path, t):
		return