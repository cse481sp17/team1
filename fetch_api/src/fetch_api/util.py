import math
import tf.transformations as tft
import numpy as np
from geometry_msgs.msg import Quaternion, Pose, Point

def convert_angle_smallest(angle):
    if angle > math.pi:
        angle = angle - 2 * math.pi
    elif angle < -math.pi:
        angle = angle + 2 * math.pi
    return angle

def quaternion_to_yaw(q):
    m = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
    x = m[0, 0] 
    y = m[1, 0]
    theta_rads = math.atan2(y, x)
    theta_degs = theta_rads * 180 / math.pi
    return theta_rads, theta_degs


def distance(x1, y1, x2, y2):
    xd = x1 - x2
    yd = y1 - y2
    return math.sqrt(xd * xd + yd * yd)

def transform_to_pose(matrix):
    pose = Pose()
    quat4 = tft.quaternion_from_matrix(matrix)
    pose.orientation = Quaternion(quat4[0], quat4[1], quat4[2], quat4[3])
    vector3 = np.dot(matrix, np.array([0, 0, 0, 1]))[:3]
    pose.position = Point(vector3[0], vector3[1], vector3[2])
    return pose

def transform(a_T_b, b_T_c):
    return transform_to_pose(np.dot(pose_to_matrix(a_T_b), pose_to_matrix(b_T_c)))

def inverse_pose(pose):
    matrix = pose_to_matrix(pose)
    inverse_transform = tft.inverse_matrix(matrix)
    return transform_to_pose(inverse_transform)

def pose_to_matrix(pose):
    o = pose.orientation
    matrix = tft.quaternion_matrix([o.x, o.y, o.z, o.w])
    matrix[0][3] = pose.position.x
    matrix[1][3] = pose.position.y
    matrix[2][3] = pose.position.z
    return matrix