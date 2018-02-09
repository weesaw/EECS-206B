#!/usr/bin/env python
"""
Starter script for lab1. 
Author: Chris Correa
"""
import copy
import rospy
import sys
import argparse

import baxter_interface
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped

# import IPython
import tf
import time
import numpy as np
from utils import *
from baxter_pykdl import baxter_kinematics
import signal
# from controllers import PDJointPositionController, PDJointVelocityController, PDJointTorqueController
from controllers2 import PDWorkspaceVelocityController, PDJointVelocityController, PDJointTorqueController
from path2 import LinearPath, CircularPath

def lookup_tag(tag_number):
    listener = tf.TransformListener()
    from_frame = 'base'
    to_frame = 'ar_marker_{}'.format(tag_number)
    # print("Base frame status: ", listener.frameExists(from_frame))
    # print("To frame status: ", listener.frameExists(to_frame))
    # if not listener.frameExists(from_frame) or not listener.frameExists(to_frame):
    #     print 'Frames not found'
    #     print 'Did you place AR marker {} within view of the baxter left hand camera?'.format(tag_number)
    #     exit(0)
    # t = listener.getLatestCommonTime(from_frame, to_frame)
    # print("The common time is: ", t)
    tag_pos = None
    
    while not tag_pos:
        try:
            t = listener.getLatestCommonTime(from_frame, to_frame)
            tag_pos, _ = listener.lookupTransform(from_frame, to_frame, t)
            break
        except:
            continue

    return tag_pos

if __name__ == "__main__":
    def sigint_handler(signal, frame):
        sys.exit(0)
    signal.signal(signal.SIGINT, sigint_handler)
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_node')
    time.sleep(1)
    
    parser = argparse.ArgumentParser()
    parser.add_argument('-ar_marker', '-ar', type=float, default=1)
    parser.add_argument('-controller', '-c', type=str, default='position') # or velocity or torque
    parser.add_argument('-arm', '-a', type=str, default='left') # or right
    args = parser.parse_args()

    limb = baxter_interface.Limb(args.arm)
    kin = baxter_kinematics('left')

    # if args.controller == 'position':
    #     # YOUR CODE HERE
    #     Kp = 
    #     Kv = 
    #     controller = PDJointPositionController(limb, kin, Kp, Kv)
    # if args.controller == 'velocity':
    #     # YOUR CODE HERE

    # #### LINEAR PATH WORKSPACE CONTROL
    # Kp = -2
    # Kv = -.7
    # controller = PDWorkspaceVelocityController(limb, kin, Kp, Kv)

    # #### CIRCULAR PATH WORKSPACE CONTROL
    # Kp = -.6
    # Kv = -0.5
    # controller = PDWorkspaceVelocityController(limb, kin, Kp, Kv)

    # ### lINEAR PATH JOINTSPACE CONTROL
    # Kp = np.array([-.8, -.5, -1, -.5, -1, -.5, -1])
    # Kv = np.array([-1.2, -.5, -.5, -.5, -.5, -.5, -.8])
    # controller = PDJointVelocityController(limb, kin, Kp, Kv)

    # ### CIRCULAR PATH JOINTSPACE CONTROL
    # Kp = np.array([-.8, -.5, -1, -.5, -1, -.5, -1])
    # Kv = np.array([-.8, -.5, -.5, -.5, -.5, -.5, -.8])
    # controller = PDJointVelocityController(limb, kin, Kp, Kv)

    Kp = np.array([-.5, -.5, -.5, -.5, -.5, -.5, -.5])
    Kv = np.array([-.5, -.5, -.5, -.5, -.5, -.5, -.5])

    # if args.controller == 'torque':
    #     # YOUR CODE HERE
    #     Kp = 
    #     Kv = 
    #     controller = PDJointTorqueController(limb, kin, Kp, Kv)
    
    raw_input('\nPlace the AR tag and press <Enter> to start')
    # YOUR CODE HERE
    # IMPORTANT: you may find useful functions in utils.py

    tag_pos = np.array(lookup_tag(4));

    # tag_pos = np.array([.6, .7, -.35])

    print("Target position: ", tag_pos)
    raw_input('AR tag found! Press <Enter> to continue')


    path1 = LinearPath(tag_pos, limb.endpoint_pose()["position"])
    # path1 = CircularPath(tag_pos, limb.endpoint_pose()["position"])
    controller = PDJointTorqueController(limb, kin, Kp, Kv)
    controller.execute_path(path1, path1.is_finished, log = True)