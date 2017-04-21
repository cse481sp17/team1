from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
class DestinationMarker(object):
    def __init__(self, server, x, y, name, driver):
        # ... Initialization, marker creation, etc. ...
        int_marker = InteractiveMarker(name=name)
        int_marker.pose.position.x = x
        int_marker.pose.position.y = y
        int_marker.pose.orientation.w = 1

        int_marker.header.frame_id = "odom"

        box_marker = Marker(type = Marker.CUBE)
        box_marker.pose.orientation.w = 1
        box_marker.scale.x = 0.45
        box_marker.scale.y = 0.45
        box_marker.scale.z = 0.45
        box_marker.color.r = 0.0
        box_marker.color.g = 0.5
        box_marker.color.b = 0.5
        box_marker.color.a = 1.0


        button_control = InteractiveMarkerControl()
        button_control.interaction_mode = InteractiveMarkerControl.BUTTON
        button_control.always_visible = True
        button_control.markers.append( box_marker )
        int_marker.controls.append( button_control )

        self._server = server
        self._server.insert(int_marker, self._callback)
        self._server.applyChanges()
        self._driver = driver

    def _callback(self, msg):
         # TODO: How do you get the interactive marker given msg.marker_name?
         # See the InteractiveMarkerServer documentation
         interactive_marker = self._server.get(msg.marker_name)
         new_position = interactive_marker.pose.position
         rospy.loginfo('User clicked {} at {}, {}, {}'.format(msg.marker_name, new_position.x, new_position.y, new_position.z))
         self._driver.goal = new_position # Updates the Driver's goal.