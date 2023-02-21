#!/usr/bin/python3

import rospy
import tf


def odom_pomocni():
	rospy.init_node('odom_pomocni')
	br = tf.TransformBroadcaster()
	listener = tf.TransformListener()
	rate = rospy.Rate(10.0)
	listener.waitForTransform("/odom", "/base_link", rospy.Time(), rospy.Duration(10.0))
	(trans,rot) = listener.lookupTransform('base_link', 'odom', rospy.Time(0))
	while not rospy.is_shutdown():
		br.sendTransform(trans, rot, rospy.Time.now(), "odom", "odom_pomocni")
		rate.sleep()

if __name__ == '__main__':
	odom_pomocni()

