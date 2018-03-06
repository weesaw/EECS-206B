#!/usr/bin/env python -W ignore::DeprecationWarning
"""
Starter script for EE106B grasp planning lab
Author: Chris Correa
"""
import numpy as np
import math
import random
import trimesh
import rospy
import tf
import time
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import Point as Point_geometry
import tf.transformations as tfs
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from autolab_core import RigidTransform, Point, NormalCloud, PointCloud
import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)
from meshpy import ObjFile, Mesh3D
warnings.filterwarnings("ignore", category=DeprecationWarning)
from visualization import Visualizer3D as vis
warnings.filterwarnings("ignore", category=DeprecationWarning)
from baxter_interface import gripper as baxter_gripper
from utils import vec, adj, create_pose_from_rigid_transform, look_at_general, look_at
import scipy
import copy
import sys
import cvxpy as cvx
import Queue
from grasp_metrics import compute_force_closure, compute_gravity_resistance, compute_custom_metric

# probably don't need to change these (but confirm that they're correct)
MAX_HAND_DISTANCE = .04
MIN_HAND_DISTANCE = .02
CONTACT_MU = 0.5
CONTACT_GAMMA = 0.1

# will need to change these
OBJECT_MASS = .25 # kg
# approximate the friction cone as the linear combination of `NUM_FACETS` vectors
NUM_FACETS = 32
# set this to false while debugging your grasp analysis
BAXTER_CONNECTED = True
# how many to execute
NUM_GRASPS = 6
# OBJECT = "pawn"
OBJECT = "nozzle"

# objects are different this year so you'll have to change this
# also you can use nodes/object_pose_publisher.py instead of finding the ar tag and then computing T_ar_object in this script.
if OBJECT == "pawn":
	MESH_FILENAME = '../objects/pawn.obj'
	# ar tag on the paper
	TAG = 14
	# transform between the object and the AR tag on the paper
	# T_ar_object = tfs.translation_matrix([-.0684, -0.099, 0.075])
	T_ar_object = tfs.translation_matrix([-.093, -0.092, .091])
	# how many times to subdivide the mesh
	SUBDIVIDE_STEPS = 0
	C_OF_MASS_HEIGHT = 0.091
elif OBJECT == "nozzle":
	MESH_FILENAME = '../objects/nozzle.obj'
	TAG = 12
	T_ar_object = tfs.translation_matrix([-.13, -.046, .032])
	SUBDIVIDE_STEPS = 1
	C_OF_MASS_HEIGHT = 0.032
elif OBJECT == "gearbox":
	MESH_FILENAME = '../objects/gearbox.obj'
	TAG = 10
	# T_ar_object = tfs.translation_matrix([-.0773, -.105, 0.056])
	T_ar_object = tfs.translation_matrix([-.072, -.102, .056])
	SUBDIVIDE_STEPS = 0
	C_OF_MASS_HEIGHT = 0.056



listener = tf.TransformListener()
from_frame = 'base'
time.sleep(1)


def lookup_tag(tag_number):
	""" Returns the AR tag position in world coordinates 

	Parameters
	----------
	tag_number : int
		AR tag number

	Returns
	-------
	:obj:`autolab_core.RigidTransform` AR tag position in world coordinates
	"""
	to_frame = 'ar_marker_{}'.format(tag_number)

	tag_pos = None
	
	while not tag_pos:
		try:
			t = listener.getLatestCommonTime(from_frame, to_frame)
			
			# print(rospy.Time.now())
			# t = rospy.Time.now()
			tag_pos, tag_rot = listener.lookupTransform(from_frame, to_frame, t)
		except:
			continue;
	# if not listener.frameExists(from_frame) or not listener.frameExists(to_frame):
	#     print 'Frames not found'
	#     print 'Did you place AR marker {} within view of the baxter left hand camera?'.format(tag_number)
	#     exit(0)
	angles = tfs.euler_from_quaternion(tag_rot)
	# print(angles)
	trans  = tfs.euler_matrix(*angles)
	print(RigidTransform(trans[:3, :3], tag_pos))
	return RigidTransform(trans[:3, :3], tag_pos)

# def get_base_gripper():

# 	to_frame = 'right_gripper_base'
# 	tag_pos = None
	
# 	while not tag_pos:
# 		try:
# 			t = listener.getLatestCommonTime(from_frame, to_frame)
# 			tag_pos, tag_rot = listener.lookupTransform(to_frame, from_frame, t)
# 		except:
# 			continue;

# 	angles = tfs.euler_from_quaternion(tag_rot)
# 	# print(angles)
# 	trans  = tfs.euler_matrix(*angles)
# 	return RigidTransform(trans[:3, :3], tag_pos)


def close_gripper():
	"""closes the gripper"""
	right_gripper.close(block=True)
	rospy.sleep(1.0)

def open_gripper():
	"""opens the gripper"""
	right_gripper.open(block=True)
	rospy.sleep(1.0)

def go_to_pose(pose):
	"""Uses Moveit to go to the pose specified
	Parameters
	----------
	pose : :obj:`geometry_msgs.msg.Pose`
		The pose to move to
	"""
	right_arm.set_start_state_to_current_state()
	# right_arm.setMaxVelocityScalingFactor(0.5)
	right_arm.set_pose_target(pose)
	right_arm.plan()
	right_arm.go()

def execute_grasp(T_object_gripper, centerTag, pawn = False,):
	"""takes in the desired hand position relative to the object, finds the desired hand position in world coordinates.  
	   Then moves the gripper from its starting orientation to some distance behind the object, then move to the 
	   hand pose in world coordinates, closes the gripper, then moves up.  
	
	Parameters
	----------
	T_object_gripper : :obj:`autolab_core.RigidTransform`
		desired position of gripper relative to the objects coordinate frame
	"""
	open_gripper()

	

	pose = create_pose_from_rigid_transform(T_object_gripper)
	inp = raw_input('Press <Enter> to move, or \'exit\' to exit')
	if inp == "exit":
		return
	# YOUR CODE HERE
	
	final_position = Point_geometry()
	final_position.x = pose.position.x
	final_position.y = pose.position.y
	final_position.z = pose.position.z

	if pawn:
		position_temp = np.dot(T_object_gripper, np.array([0, 0, -0.1, 1]))
		pose.position.x = position_temp[0]
		pose.position.y = position_temp[1]
		pose.position.z = position_temp[2]


	go_to_pose(pose)

	if pawn:
		print("Going towards the object")
		rospy.sleep(1.0)
		pose.position = final_position
		go_to_pose(pose)


	rospy.sleep(3.0)
	close_gripper()




	pose = create_pose_from_rigid_transform(centerTag)

	pose.position.z = pose.position.z + 0.2
	go_to_pose(pose)
	rospy.sleep(3.0)
	pose.position.z = pose.position.z - 0.2
	go_to_pose(pose)
	open_gripper()

def contacts_to_baxter_hand_pose(contact1, contact2, pawn = False ,approach_direction=None):
	""" takes the contacts positions in the object frame and returns the hand pose T_obj_gripper
	
	Parameters
	----------
	contact1 : :obj:`numpy.ndarray`
		position of finger1 in object frame
	contact2 : :obj:`numpy.ndarray`
		position of finger2 in object frame
	approach_direction : :obj:`numpy.ndarray`
		there are multiple grasps that go through contact1 and contact2.  This describes which 
		orientation the hand should be in

	Returns
	-------
	:obj:`autolab_core:RigidTransform` Hand pose in the object frame
	"""
	# YOUR CODE HERE
	midFrame = (contact1 + contact2)/2
	line  = contact2 - contact1
	
	if pawn:
		# up = np.array([math.sqrt(2)/2, 0, math.sqrt(2)/2])
		up = np.array([1, 0, 0])
	else:
		up = np.array([0, 0, 1])
	# R = look_at(line, up)
	# p = midFrame - 
	return look_at_general(midFrame, line, up)




	

	# T_obj_gripper = ????
	# return T_obj_gripper

def sorted_contacts(vertices, normals, triangles, T_ar_object, metric = compute_force_closure):
	""" takes mesh and returns pairs of contacts and the quality of grasp between the contacts, sorted by quality
	
	Parameters
	----------
	vertices : :obj:`numpy.ndarray`
		nx3 mesh vertices
	normals : :obj:`numpy.ndarray`
		nx3 mesh normals
	T_ar_object : :obj:`autolab_core.RigidTransform`
		transform from the AR tag on the paper to the object

	Returns
	-------
	:obj:`list` of :obj:`numpy.ndarray`
		grasp_indices[i][0] and grasp_indices[i][1] are the indices of a pair of vertices.  These are randomly 
		sampled and their quality is saved in best_metric_indices
	:obj:`list` of int
		best_metric_indices is the indices of grasp_indices in order of grasp quality
	"""
	
	# prune vertices that are too close to the table so you dont smack into the table
	# you may want to change this line, to be how you see fit
	# print(vertices[:5])
	possible_indices = np.r_[:len(vertices)][vertices[:,2] + T_ar_object[2,3] >= 0.02]

	print metric

	# Finding grasp via vertex sampling.  make sure to not consider grasps where the 
	# vertices are too big for the gripper
	# alpha = np.arctan2(CONTACT_MU, 1)
	# print(alpha)
	grasp_indices = list()
	best_metric_indices = list()
	best_metrics = list()

	while len(grasp_indices) < 100:
		i, j = np.random.choice(possible_indices, 2, replace=False)
		l = vertices[i] - vertices[j]
		if np.linalg.norm(l) < MAX_HAND_DISTANCE and np.linalg.norm(l) > MIN_HAND_DISTANCE:
			# and abs(vertices[i_index[0]][2] - vertices[i_index[1]][2]) < 0.02:
			contacts = [vertices[i], vertices[j]]

			# if contacts[0][2] > 0.05-0.02:
			# 	if contacts[0][0]<-0.025 and contacts[0][0]>-0.06:
			# 		continue

			# if contacts[1][2] > 0.05-0.032:
			# 	if contacts[1][0]<-0.025 and contacts[1][0]>-0.06:
			# 		continue

			contact_normals = [normals[i], normals[j]]
			quality = metric(contacts, contact_normals, NUM_FACETS, CONTACT_MU, CONTACT_GAMMA, OBJECT_MASS, 0.1)
			if quality > 1000:
				continue

			index = np.array([i,j])
			grasp_indices.append(index)
			best_metric_indices.append(index)
			best_metrics.append(quality)
			# Old force closure
			# n1 = normals[i_index[0]]
			# n2 = normals[i_index[1]]
			# theta1 = np.arccos(np.dot(n1, l)/np.linalg.norm(l));
			
			# if abs(theta1)>alpha:
			# 	continue;
			# theta2 = np.pi - np.arccos(np.dot(n2, l)/np.linalg.norm(l));
			
			# if abs(theta2)>alpha:
			# 	continue;

			

			
			# print len(grasp_indices)
	# YOUR CODE HERE.  sort metrics and return the sorted order
	sorted_indices = sorted(zip(best_metrics, best_metric_indices), key=lambda x: x[0])
	best_metric_indices = zip(*sorted_indices)[1]

	return grasp_indices, best_metric_indices


if __name__ == '__main__':
	if BAXTER_CONNECTED:
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('moveit_node')
		robot = moveit_commander.RobotCommander()
		scene = moveit_commander.PlanningSceneInterface()
		right_arm = moveit_commander.MoveGroupCommander('right_arm')
		# right_arm = moveit_commander.MoveGroupCommander('left_arm')
		# right_arm.set_planner_id('RRTConnectkConfigDefault')
		right_arm.set_planning_time(5)
		right_gripper = baxter_gripper.Gripper('right')
		# right_gripper = baxter_gripper.Gripper('left')
		tag = lookup_tag(TAG)
		
	# o = True
	# while True:
	# 	inp = raw_input('Press <Enter> om.randint(0, len(indeces)-1)

		# normal1 = normals[indeces[i][0]]
		# normal2 = normals[indeces[i][1]]
		

		# contact1 = vertices[indeces[i][0]]
		# contactto print, or \'exit\' to exit')
	# 	if inp == "exit":
	# 		break;
	

	# #For simulation
	rospy.init_node('moveit_node')

	centerTag = lookup_tag(14)

	
	# Main Code
	br = tf.TransformBroadcaster()

	

	# We found this helped.  You may not.  I believe there was a problem with setting the surface normals.
	# I remember fixing that....but I didn't save that code, so you may have to redo it.  
	# You may need to fix that if you call this function.
	mesh = trimesh.load(MESH_FILENAME)

	for i in range(SUBDIVIDE_STEPS):
		mesh = mesh.subdivide()

	vertices = mesh.vertices
	triangles = mesh.triangles
	normals = mesh.vertex_normals
	if OBJECT == "pawn":
		normals = -normals
	print normals.shape, vertices.shape


	# SETUP
	of = ObjFile(MESH_FILENAME)
	mesh = of.read()
	# mesh = Mesh3D(vertices, triangles, normals)

	_, indeces = sorted_contacts(vertices, normals, triangles, T_ar_object, metric = compute_custom_metric)
	

	# # print(vertices[:5])
	# # print(normals[:5])
	# # print(indeces[:5]) 
	# # print(type(indeces))
	# # print(type(normals))

	# print(len(indeces))

	# YOUR CODE HERE
	repeat = True
	i = 0

	while repeat:
		# i = random.randint(0, len(indeces)-1)

		normal1 = normals[indeces[i][0]]
		normal2 = normals[indeces[i][1]]
		

		contact1 = vertices[indeces[i][0]]
		contact2 = vertices[indeces[i][1]]
		
		i=i+1
		# #####For center of mass test
		# contact1 = np.array([0, .015, 0])
		# contact2 = np.array([0, -.015, 0])
		# normal1 = normals[0]
		# normal2 = normals[1]
		


		# visualize the mesh and contacts
		vis.figure()
		vis.mesh(mesh)
		vis.normals(NormalCloud(np.hstack((normal1.reshape(-1, 1), normal2.reshape(-1, 1))), frame='test'),
			PointCloud(np.hstack((contact1.reshape(-1, 1), contact2.reshape(-1, 1))), frame='test'))
		# vis.pose(T_obj_gripper, alpha=0.05)
		vis.show()
		done_looking = True if raw_input("Execute grasp?") == "y" else False

		if done_looking and BAXTER_CONNECTED:
			repeat_grasp = True
			while repeat_grasp:
				open_gripper()
				
				T = contacts_to_baxter_hand_pose(contact1, contact2, pawn = False)
				# T = np.dot(tfs.euler_matrix(math.pi, 0, 0), T)
				T2 = np.dot(T_ar_object, T)
				fullTrans = np.dot(tag.matrix, T2)
				
				rot = RigidTransform(np.array([[0, -1, 0], [1, 0, 0], [0,0,1]]), np.array([0,0,0]))
				newPos = np.dot(rot.matrix, T2)
				centerFull = np.dot(centerTag.matrix, newPos)
				


				br = tf.TransformBroadcaster()
				br.sendTransform(tfs.translation_from_matrix(fullTrans), tfs.quaternion_from_matrix(fullTrans),
						rospy.Time(0),
						"object",
						"base")
				br.sendTransform(tfs.translation_from_matrix(centerFull), tfs.quaternion_from_matrix(centerFull),
						rospy.Time(0),
						"centerFull",
						"base")
				
				execute_grasp(fullTrans ,centerFull, pawn = True)
				
				# close_gripper()

				repeat_grasp = bool(raw_input("repeat grasp?"))

		repeat = True if raw_input("Next set of contacts?") == "y" else False
	# 500, 1200
	exit()
