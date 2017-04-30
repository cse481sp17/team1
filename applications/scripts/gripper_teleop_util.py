from visualization_msgs.msg import *
from std_msgs.msg import ColorRGBA
import rospy, copy

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

def create_gripper_marker(pose):
    # Creates interactive marker with metadata, pose
    gripper_marker = InteractiveMarker()
    gripper_marker.header.frame_id = 'base_link' # TODO: change to wrist_roll_link after done
    gripper_marker.description = 'Interactive Marker for Gripper'
    gripper_marker.name = 'gripper_im'
    gripper_marker.pose = pose
    gripper_marker.scale = 0.5

    # Create marker for gripper base
    base_marker = Marker()
    base_marker.type = Marker.MESH_RESOURCE # TODO: if this doesn't work, then set it to 10
    base_marker.mesh_resource = GRIPPER_MESH
    base_marker.pose.position.x += 0.166

    # Create marker for left gripper prong
    left_marker = Marker()
    left_marker.type = Marker.MESH_RESOURCE
    left_marker.mesh_resource = L_FINGER_MESH
    left_marker.scale.y = 0.6
    left_marker.color = RED
    left_marker.pose.position.x += 0.166

    # Create marker for right gripper prong
    right_marker = Marker()
    right_marker.type = Marker.MESH_RESOURCE
    right_marker.mesh_resource = R_FINGER_MESH
    right_marker.scale.y = 0.6
    right_marker.color = RED
    right_marker.pose.position.x += 0.166

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


    gripper_control.markers.append(base_marker)
    gripper_control.markers.append(left_marker)
    gripper_control.markers.append(right_marker)

    # Add the above control to the interactive marker
    gripper_marker.controls.append(gripper_control)
    gripper_marker.controls.extend(make_6dof_controls())

    # Return the InteractiveMarker
    return gripper_marker

def make_6dof_controls():
    controls = []
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
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
    control.name = "rotate_y"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    controls.append(copy.deepcopy(control))
    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    controls.append(control) # not deep copy
    return controls