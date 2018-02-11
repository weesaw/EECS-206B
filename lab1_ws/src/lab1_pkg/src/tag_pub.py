#!/usr/bin/env python
<<<<<<< HEAD
import rospy
import numpy as np
from geometry_msgs.msg import Point
import argparse
import tf


def lookup_tag(tag_number):
		listener = tf.TransformListener()
		from_frame = 'base'
		to_frame = 'ar_marker_{}'.format(tag_number)
		tag_pos = None
		
		while not tag_pos:
			try:
				t = listener.getLatestCommonTime(from_frame, to_frame)
				tag_pos, _ = listener.lookupTransform(from_frame, to_frame, t)
			except:
				continue
		
		print(tag_number, "found:", tag_pos)

		return tag_pos

def tag_pub(tag_number):
	rospy.init_node('tag_pub', anonymous=True)
	pub = rospy.Publisher('tag_talk', Point, queue_size=10)
	r = rospy.Rate(100) # 10hz

	listener = tf.TransformListener()
	from_frame = 'base'
	to_frame = 'ar_marker_{}'.format(tag_number)
	tag_pos = None

	while not rospy.is_shutdown():
		# pub_point = lookup_tag(tag_number)
		while not tag_pos:
			try:
				t = listener.getLatestCommonTime(from_frame, to_frame)
				tag_pos, _ = listener.lookupTransform(from_frame, to_frame, t)
			except:
				continue
		pub.publish(*tag_pos)
		print(tag_number, "found:", tag_pos)
		tag_pos = None
		r.sleep()
			
if __name__ == '__main__':
		parser = argparse.ArgumentParser()
		parser.add_argument('tag_number', type=int, help='the tag number')
		args = parser.parse_args()
		try:
			tag_pub(args.tag_number)
		except rospy.ROSInterruptException: pass
=======
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy
import numpy as np

#Import the String message type from the /msg directory of
#the std_msgs package.
from geometry_msgs.msg import Point


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
    
    print(tag_number, "found:", tag_pos)
    return np.array(tag_pos)

#Define the method which contains the main functionality of the node.
def talker():

  #Run this program as a new node in the ROS computation graph 
  #called /talker.
  rospy.init_node('tag_pos', anonymous=True)

  #Create an instance of the rospy.Publisher object which we can 
  #use to publish messages to a topic. This publisher publishes 
  #messages of type std_msgs/String to the topic /chatter_talk
  pub = rospy.Publisher('artag_talk', Point, queue_size=10)
  
  # Create a timer object that will sleep long enough to result in
  # a 10Hz publishing rate
  r = rospy.Rate(10) # 10hz

  # Loop until the node is killed with Ctrl-C
  while not rospy.is_shutdown():
    # Construct a string that we want to publish
    # (In Python, the "%" operator functions similarly
    #  to sprintf in C or MATLAB)
    # pub_string = "hello world %s" % (rospy.get_time())
    pub_point = lookup_tag(1)


    # Publish our string to the 'chatter_talk' topic
    pub.publish(pub_point)
    
    # Use our rate object to sleep until it is time to publish again
    r.sleep()
      
# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  # Check if the node has received a signal to shut down
  # If not, run the talker method
  try:
    talker()
  except rospy.ROSInterruptException: pass
>>>>>>> bb09561cf45fbaa5f5a6efa2508fa3e8f13fd13a
