#!/usr/bin/env python  
import roslib
roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('tf_fetch')
    listener = tf.TransformListener()
    rospy.sleep(0.1)
    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        try:
            print 'gripper_link ', listener.lookupTransform('/base_link', '/gripper_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()