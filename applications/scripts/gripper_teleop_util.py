import numpy as np
import tf.transformations as tft
import rospy, copy
from visualization_msgs.msg import *
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped

GRIPPER_POSE_ID = 1
OPEN_GRIPPER_ID = 2
CLOSE_GRIPPER_ID = 3

GRIPPER_MESH = 'package://fetch_description/meshes/gripper_link.dae'
L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.STL'

RED = ColorRGBA()
RED.r = 1.0
RED.g = 0.0
RED.b = 0.0
RED.a = 1.0

GREEN = ColorRGBA()
GREEN.r = 0.0
GREEN.g = 1.0
GREEN.b = 0.0
GREEN.a = 1.0

GRIPPER_SCALE = 0.5
GRIPPER_X_OFFSET = 0.166
FINGER_Y_SCALE = 0.6
PREGRASP_OFFSET = -0.15
LIFT_OFFSET = 0.4

def pose_to_matrix(pose):
    o = pose.orientation
    matrix = tft.quaternion_matrix([o.x, o.y, o.z, o.w])
    matrix[0][3] = pose.position.x
    matrix[1][3] = pose.position.y
    matrix[2][3] = pose.position.z
    return matrix

def matrix_to_pose(matrix):
    pose = Pose()
    quat4 = tft.quaternion_from_matrix(matrix)
    pose.orientation = Quaternion(quat4[0], quat4[1], quat4[2], quat4[3])
    vector3 = np.dot(matrix, np.array([0, 0, 0, 1]))[:3]
    pose.position = Point(vector3[0], vector3[1], vector3[2])
    return pose

def a_to_frame_b(matrix_a, matrix_b):
    return np.dot(matrix_a, matrix_b)

def pose_in_marker_frame(marker, pose):
    pose_marker = copy.deepcopy(marker).pose
    pose_marker.position.x -= GRIPPER_X_OFFSET
    return matrix_to_pose(a_to_frame_b(pose_to_matrix(pose), pose_to_matrix(pose_marker)))

def create_pose_stamped(pose):
    ps = PoseStamped()
    ps.header.frame_id = 'base_link'
    ps.header.stamp = rospy.get_rostime()
    ps.pose = copy.deepcopy(pose)
    return ps

def create_gripper_marker(x, z):
    # Create marker for gripper base
    base_marker = Marker()
    base_marker.type = Marker.MESH_RESOURCE
    base_marker.mesh_resource = GRIPPER_MESH
    base_marker.color = RED
    base_marker.pose.position.x += x + GRIPPER_X_OFFSET
    base_marker.pose.position.z += z

    # Create marker for left gripper prong
    left_marker = copy.deepcopy(base_marker)
    left_marker.mesh_resource = L_FINGER_MESH
    left_marker.scale.y = FINGER_Y_SCALE

    # Create marker for right gripper prong
    right_marker = copy.deepcopy(left_marker)
    right_marker.mesh_resource = R_FINGER_MESH

    return [base_marker, left_marker, right_marker]

def create_gripper_interactive_marker(pose, pregrasp=True, rotation_enabled=True):
    # Creates interactive marker with metadata, pose
    gripper_marker = InteractiveMarker()
    gripper_marker.header.frame_id = 'base_link'
    gripper_marker.description = 'Interactive Marker for Gripper'
    gripper_marker.name = 'gripper_im'
    gripper_marker.pose = pose
    gripper_marker.scale = GRIPPER_SCALE

    # Create a which controls the 3 gripper components
    gripper_control = InteractiveMarkerControl()
    gripper_control.interaction_mode = InteractiveMarkerControl.MENU # or InteractiveMarkerControl.MOVE_ROTATE
    gripper_control.always_visible = True
    gripper_control.orientation.w = 1
    gripper_control.orientation.x = 0
    gripper_control.orientation.y = 1
    gripper_control.orientation.z = 0

    # gripper pose Menu Entry
    menu_entry = MenuEntry()
    menu_entry.id = GRIPPER_POSE_ID
    menu_entry.parent_id = 0
    menu_entry.command_type = MenuEntry.FEEDBACK
    menu_entry.title = 'go to the gripper pose' # make constants
    gripper_marker.menu_entries.append(copy.deepcopy(menu_entry))
    # open the gripper menu entry
    menu_entry.id = OPEN_GRIPPER_ID
    menu_entry.parent_id = 0
    menu_entry.command_type = MenuEntry.FEEDBACK
    menu_entry.title = 'open the gripper'
    gripper_marker.menu_entries.append(copy.deepcopy(menu_entry))
    # close the gripper menu entry 
    menu_entry.id = CLOSE_GRIPPER_ID
    menu_entry.parent_id = 0
    menu_entry.command_type = MenuEntry.FEEDBACK
    menu_entry.title = 'close the gripper'
    gripper_marker.menu_entries.append(menu_entry) # not deep copy

    gripper_control.markers.extend(create_gripper_marker(0, 0))
    if pregrasp:
        gripper_control.markers.extend(create_gripper_marker(PREGRASP_OFFSET, 0))
        gripper_control.markers.extend(create_gripper_marker(0, LIFT_OFFSET))

    # Add the above control to the interactive marker
    gripper_marker.controls.append(gripper_control)
    gripper_marker.controls.extend(make_6dof_controls(rotation_enabled))

    # Return the InteractiveMarker
    return gripper_marker

def make_6dof_controls(rotation_enabled):
    controls = []
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    if rotation_enabled:
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        controls.append(copy.deepcopy(control))
    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    controls.append(copy.deepcopy(control))

    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    if rotation_enabled:
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        controls.append(copy.deepcopy(control))
    control.name = "move_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    controls.append(copy.deepcopy(control))

    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    if rotation_enabled:
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        controls.append(copy.deepcopy(control))
    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    controls.append(control) # not deep copy
    return controls
