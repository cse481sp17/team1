#! /usr/bin/env python

import math
import numpy as np
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from std_msgs.msg import ColorRGBA
import visualization_msgs.msg
import rospy
import tf.transformations as tft

def transform_to_pose(matrix):
    pose = Pose()
    quat4 = tft.quaternion_from_matrix(matrix)
    pose.orientation = Quaternion(quat4[0], quat4[1], quat4[2], quat4[3])
    vector3 = np.dot(matrix, np.array([0, 0, 0, 1]))[:3]
    pose.position = Point(vector3[0], vector3[1], vector3[2])
    return pose

if __name__ == "__main__":

    object_T_pregrasp = tft.identity_matrix()
    object_T_pregrasp[0][3] = -0.1

    base_link_T_object =  tft.quaternion_matrix([0, 0, 0.38268343, 0.92387953])
    base_link_T_object[0][3] = 0.6
    base_link_T_object[1][3] = -0.1
    base_link_T_object[2][3] = 0.7
    print 'base_link_T_object', base_link_T_object
    print(transform_to_pose(np.dot(base_link_T_object, object_T_pregrasp)))
