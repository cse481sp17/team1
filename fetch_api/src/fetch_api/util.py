import math
import tf.transformations as tft
import numpy as np
from geometry_msgs.msg import Quaternion

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
