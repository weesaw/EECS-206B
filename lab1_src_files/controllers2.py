import rospy
import numpy as np
from utils import *
from geometry_msgs.msg import PoseStamped, Point
import time
from baxter_core_msgs.msg import SEAJointState

"""
Starter script for lab1. 
Author: Chris Correa
"""
class Controller:
	# joint_order = ["left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"]
	joint_order = ["right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2"]
	ar_tag = None


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
		if isinstance(self, PDJointVelocityController) or isinstance(self, PDJointTorqueController):
			actual_thetas = list()
			target_thetas = list()
			actual_theta_dots = list()
			target_theta_dots = list()
		r = rospy.Rate(200)


		# rospy.init_node('artag_listner', anonymous=True)
		# rospy.Subscriber('artag_talk', Point, lambda x: ar_tag = x.data)
		# rospy.spin()

		while True:
			t = (rospy.Time.now() - start_t).to_sec()
			if timeout is not None and t >= timeout:
				break
			success = self.step_path(path, t)
			if log:
				times.append(t)
				full_pos = self.limb.endpoint_pose()["position"]
				actual_positions.append([full_pos[0], full_pos[1], full_pos[2]])
				full_vel = self.limb.endpoint_velocity()["linear"]
				actual_velocities.append([full_vel[0], full_vel[1], full_vel[2]])

				target_positions.append(path.target_position(t).tolist())
				target_velocities.append(path.target_velocity(t).tolist())
				joint_thetas_dict = self.limb.joint_angles()
				joint_velocities_dict = self.limb.joint_velocities()

				if isinstance(self, PDJointVelocityController) or isinstance(self, PDJointTorqueController):
					actual_thetas.append([joint_thetas_dict[k] for k in Controller.joint_order])
					actual_theta_dots.append([joint_velocities_dict[k] for k in Controller.joint_order])
					target_thetas.append(self.target_thetas.tolist())
					target_theta_dots.append(self.target_theta_dots.tolist())
			# print(path)
			check_pos = self.limb.endpoint_pose()["position"]

			#ag_pos = lookup_tag()
			# if finished is not None and finished(check_pos, t) or success == -1:
			# 	if success == -1:
			# 		print("Inverse kinematics not working!")
			# 	break
			# r.sleep()

			if finished is not None and finished(check_pos, t) or success == -1:
				if success == -1:
					print("Inverse kinematics not working!")
				break
			r.sleep()

		if log:
			import matplotlib.pyplot as plt
			
			actual_velocities = np.reshape(np.array(actual_velocities), (len(times), 3))
			target_velocities = np.reshape(np.array(target_velocities), (len(times),3))
			actual_positions = np.reshape(np.array(actual_positions), (len(times), 3))
			target_positions = np.reshape(np.array(target_positions), (len(times),3))

			plt.subplot(3,2,1)
			plt.plot(times, actual_velocities[:,0], label="X Velocity actual")
			plt.plot(times, target_velocities[:,0], label="X Velocity target")
			plt.legend()

			plt.subplot(3,2,2)
			plt.plot(times, actual_velocities[:,1], label="Y Velocity actual")
			plt.plot(times, target_velocities[:,1], label="Y Velocity target")
			plt.legend()

			plt.subplot(3,2,3)
			plt.plot(times, target_velocities[:,2], label="Z Velocity target")
			plt.plot(times, actual_velocities[:,2], label="Z Velocity actual")
			plt.legend()

			plt.subplot(3,2,4)
			plt.plot(times, actual_positions[:,0], label="X Position actual")
			plt.plot(times, target_positions[:,0], label="X Position target")
			plt.legend()

			plt.subplot(3,2,5)
			plt.plot(times, actual_positions[:,1], label="Y Position actual")
			plt.plot(times, target_positions[:,1], label="Y Position target")
			plt.legend()

			plt.subplot(3,2,6)
			plt.plot(times, target_positions[:,2], label="Z Position target")
			plt.plot(times, actual_positions[:,2], label="Z Position actual")
			plt.legend()
			
			if isinstance(self, PDJointVelocityController) or isinstance(self, PDJointTorqueController):
				actual_thetas = np.reshape(actual_thetas, (len(times), 7))
				target_thetas = np.reshape(target_thetas, (len(times), 7))
				actual_theta_dots = np.reshape(actual_theta_dots, (len(times), 7))
				target_theta_dots = np.reshape(target_theta_dots, (len(times), 7))

				plt.figure()
				for i in range(1,8):
					plt.subplot(7,2,i)
					plt.plot(times, actual_thetas[:,i-1], label="J{} actual theta".format(i))
					plt.plot(times, target_thetas[:,i-1], label="J{} target theta".format(i))
					plt.legend()

				for i in range(8,15):
					plt.subplot(7,2,i)
					plt.plot(times, actual_theta_dots[:,i-8], label="J{} actual theta dot".format(i-7))
					plt.plot(times, target_theta_dots[:,i-8], label="J{} target theta dot".format(i-7))
					plt.legend()


			plt.show()

		# if log:
		# 	import matplotlib.pyplot as plt

		# 	np_actual_positions = np.zeros((len(times), 3))
		# 	np_actual_velocities = np.zeros((len(times), 3))
		# 	for i in range(len(times)):
		# 		print(actual_positions)
		# 		actual_positions_dict = dict((joint, actual_positions[i][j]) for j, joint in enumerate(self.limb.joint_names()))
		# 		print "dictionary version", actual_positions_dict
		# 		np_actual_positions[i] = self.kin.forward_position_kinematics(joint_values=actual_positions_dict)[:3]
		# 		np_actual_velocities[i] = self.kin.jacobian(joint_values=actual_positions_dict)[:3].dot(actual_velocities[i])
		# 	target_positions = np.array(target_positions)
		# 	target_velocities = np.array(target_velocities)
		# 	plt.figure()
		# 	# print len(times), actual_positions.shape()
		# 	plt.subplot(3,2,1)
		# 	plt.plot(times, np_actual_positions[:,0], label='Actual')
		# 	plt.plot(times, target_positions[:,0], label='Desired')
		# 	plt.xlabel("Time (t)")
		# 	plt.ylabel("X Position Error")

		# 	plt.subplot(3,2,2)
		# 	plt.plot(times, np_actual_velocities[:,0], label='Actual')
		# 	plt.plot(times, target_velocities[:,0], label='Desired')
		# 	plt.xlabel("Time (t)")
		# 	plt.ylabel("X Velocity Error")
			
		# 	plt.subplot(3,2,3)
		# 	plt.plot(times, np_actual_positions[:,1], label='Actual')
		# 	plt.plot(times, target_positions[:,1], label='Desired')
		# 	plt.xlabel("time (t)")
		# 	plt.ylabel("Y Position Error")

		# 	plt.subplot(3,2,4)
		# 	plt.plot(times, np_actual_velocities[:,1], label='Actual')
		# 	plt.plot(times, target_velocities[:,1], label='Desired')
		# 	plt.xlabel("Time (t)")
		# 	plt.ylabel("Y Velocity Error")
			
		# 	plt.subplot(3,2,5)
		# 	plt.plot(times, np_actual_positions[:,2], label='Actual')
		# 	plt.plot(times, target_positions[:,2], label='Desired')
		# 	plt.xlabel("time (t)")
		# 	plt.ylabel("Z Position Error")

		# 	plt.subplot(3,2,6)
		# 	plt.plot(times, np_actual_velocities[:,2], label='Actual')
		# 	plt.plot(times, target_velocities[:,2], label='Desired')
		# 	plt.xlabel("Time (t)")
		# 	plt.ylabel("Z Velocity Error")

		# 	plt.show()

		return True

class PDWorkspaceVelocityController(Controller):
	def __init__(self, limb, kin, Kp, Kv):
		self.limb = limb
		self.kin = kin
		self.Kp = Kp
		self.Kv = Kv

	def step_path(self, path, time):
		error = self.limb.endpoint_pose()['position'] - path.target_position(time)
		error_dot = self.limb.endpoint_velocity()['linear'] - path.target_velocity(time)
		new_vel = self.Kp*error + self.Kv*error_dot

		# print(error, error_dot)

		new_vel = np.append(new_vel, np.zeros((3,1)))
		joint_vels = np.dot(self.kin.jacobian_pseudo_inverse(), new_vel)

		# print(joint_vels)

		dict_ = {}
		keys = self.limb.joint_names()
		for i in range(len(keys)):
			dict_[keys[i]] = joint_vels.item(i)

		self.limb.set_joint_velocities(dict_)



class PDJointVelocityController(Controller):
	def __init__(self, limb, kin, Kp, Kv):
		self.limb = limb
		self.kin = kin
		self.Kp = Kp
		self.Kv = Kv
		self.target_thetas = None
		self.target_theta_dots = None

	def step_path(self, path, t):
		# dict_vels = self.limb.joint_velocities()
		dict_pos = self.limb.joint_angles()
		# current_joint_vel = np.array([dict_vels[k] for k in ["left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"]])
		current_joint_pos = np.array([dict_pos[k] for k in Controller.joint_order])

		current_orein = self.limb.endpoint_pose()["orientation"]

		temp = self.kin.inverse_kinematics(path.target_position(t), current_orein)
		if temp is None and t != 0:
			return -1
		self.target_thetas = temp
		
		joint_error = current_joint_pos - self.target_thetas

		self.target_theta_dots = np.dot(self.kin.jacobian_pseudo_inverse(), np.append(path.target_velocity(t), np.zeros((3,1))))
		
		workspace_error_dot = self.limb.endpoint_velocity()["linear"] - path.target_velocity(t)
		joint_error_dot = np.dot(self.kin.jacobian_pseudo_inverse(), np.append(workspace_error_dot,np.zeros((3,1))))

		new_vel = np.multiply(self.Kp, joint_error) + np.multiply(self.Kv, joint_error_dot)

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

		self.target_theta_dots = None
		self.target_thetas = None


		# print(self.init_torque)
		# self.new_torque = None
		self.prev_J_inv = self.kin.jacobian_pseudo_inverse()
		self.prev_time = rospy.Time.now().to_sec()


	def step_path(self, path, t):
		self.target_theta_dots = np.dot(self.kin.jacobian_pseudo_inverse(), np.append(path.target_velocity(t), np.zeros((3,1))))

		dict_pos = self.limb.joint_angles()
		current_joint_pos = np.array([dict_pos[k] for k in Controller.joint_order])

		current_orein = self.limb.endpoint_pose()["orientation"]

		temp = self.kin.inverse_kinematics(path.target_position(t), current_orein)
		if temp is None and t != 0:
			return -1
		self.target_thetas = temp

		joint_error = current_joint_pos - self.target_thetas


		workspace_error_dot = self.limb.endpoint_velocity()["linear"] - path.target_velocity(t)
		joint_error_dot = np.dot(self.kin.jacobian_pseudo_inverse(), np.append(workspace_error_dot,np.zeros((3,1))))


		dt = rospy.Time.now().to_sec() - self.prev_time
		self.prev_time = rospy.Time.now().to_sec()

		# target_theta_ddot = np.dot(self.kin.jacobian_pseudo_inverse(), path.target_acceleration(t)) \
		# 				   +np.reshape(np.dot((self.kin.jacobian_pseudo_inverse() - self.prev_J_inv)/dt, np.append(path.target_velocity(t), np.zeros((3,1)))), (7,1))

		target_theta_ddot = np.dot(self.kin.jacobian_pseudo_inverse(), path.target_acceleration(t)) + np.dot((self.kin.jacobian_pseudo_inverse() - self.prev_J_inv)/dt, np.append(path.target_velocity(t), np.zeros((3,1))))

		self.prev_J_inv = self.kin.jacobian_pseudo_inverse()


		# M_term = np.dot(self.kin.inertia(), np.reshape(target_theta_ddot, (7,1)))
		M_term = np.dot(self.kin.inertia(), target_theta_ddot)


		new_torque = M_term + np.multiply(self.Kp, joint_error) + np.multiply(self.Kv, joint_error_dot)
		# print(new_torque)
		dict_ = {}
		for i in range(len(Controller.joint_order)):
			dict_[Controller.joint_order[i]] = new_torque.item(i)

		# print(new_vel)

		self.limb.set_joint_torques(dict_)