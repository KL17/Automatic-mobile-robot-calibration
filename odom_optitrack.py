#!/usr/bin/python3

import rospy
import tf


def odom_optitrack():
	rospy.init_node('optitrack')
	br = tf.TransformBroadcaster()
	listener = tf.TransformListener()
	rate = rospy.Rate(10.0)
	listener.waitForTransform("/world", "/robot", rospy.Time(), rospy.Duration(10))
	(trans,rot) = listener.lookupTransform('robot', 'world', rospy.Time(0))
	while not rospy.is_shutdown():
		br.sendTransform(trans, rot, rospy.Time.now(), "world", "odom_optitrack")
		rate.sleep()
		
if __name__ == '__main__':
	odom_optitrack()

