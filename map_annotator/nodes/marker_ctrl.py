import rospy
from visualization_msgs.msg import *

class MarkerController(object):
    def __init__(self, server, pose_ctrl):
        self._server = server
        self._pose_ctrl = pose_ctrl
        for name, pose in self._pose_ctrl.poses.items():
            self.create_marker(name, pose.pose.pose)

    def _marker_callback(self, msg):
        if (msg.event_type == InteractiveMarkerFeedback.POSE_UPDATE):
            self._server.setPose(msg.marker_name, msg.pose)
            self._server.applyChanges()
            self._pose_ctrl.update_pose(msg.marker_name, msg.pose)
            rospy.logwarn("Updated marker for pose: {}".format(msg.marker_name))

    def create_marker(self, name, pose):
        # Creates interactive marker with metadata, pose
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.name = name
        int_marker.description = "Pose: {}".format(name)
        int_marker.pose = pose
        int_marker.pose.position.z = 0.5

        # Create arrow and circle
        arrow_marker = Marker()
        arrow_marker.type = Marker.ARROW
        arrow_marker.scale.x = 0.5    # put it in place
        arrow_marker.scale.y = 0.1
        arrow_marker.scale.z = 0.1
        arrow_marker.color.r = 0.5    # make it pretty
        arrow_marker.color.g = 0.0
        arrow_marker.color.b = 0.5
        arrow_marker.color.a = 1.0

        # Create a non-interactive control which controls the arrow
        arrow_control = InteractiveMarkerControl()
        arrow_control.always_visible = True
        arrow_control.markers.append(arrow_marker)

        rotate_control = InteractiveMarkerControl()
        rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE
        rotate_control.orientation.w = 1
        rotate_control.orientation.x = 0
        rotate_control.orientation.y = 1
        rotate_control.orientation.z = 0

        # Add the above control to the interactive marker
        int_marker.controls.append(arrow_control)
        int_marker.controls.append(rotate_control)

        # Apply changes to our marker server
        self._server.insert(int_marker, self._marker_callback)
        self._server.applyChanges()

        rospy.logwarn("Created marker for pose: {}".format(name))

    def erase_marker(self, name):
        self._server.erase(name)
        self._server.applyChanges()
        rospy.logwarn("Erased marker for pose: {}".format(name))

    def rename_marker(self, name, name_new):
        self.create_marker(name_new, self._pose_ctrl.poses[name].pose.pose)
        self.erase_marker(name)

