#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Point
import argparse
import tf


# def lookup_tag(tag_number):
# 		listener = tf.TransformListener()
# 		from_frame = 'base'
# 		to_frame = 'ar_marker_{}'.format(tag_number)
# 		tag_pos = None
		
# 		while not tag_pos:
# 			try:
# 				t = listener.getLatestCommonTime(from_frame, to_frame)
# 				tag_pos, _ = listener.lookupTransform(from_frame, to_frame, t)
# 			except:
# 				continue
		
# 		print(tag_number, "found:", tag_pos)

# 		return tag_pos

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
