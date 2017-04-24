#!/usr/bin/env python
import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
from map_annotator.msg import UserAction, PoseNames

PUB_NAME = "/user_actions"
POSE_NAME = "/pose_names"

class InteractiveMarkerMonitor(object):
    def __init__(self, marker_server):
        self._marker_server = marker_server
        self._sub = rospy.Subscriber(POSE_NAME, PoseNames, self._callback)
        self._pub = rospy.Publisher(PUB_NAME, UserAction, queue_size=10, latch=True)
    
    def _callback(self, msg):
        print msg
        print 'in callback'
        # Create an interactive marker for each pose in our list
        for pose_name in msg.names:
            print pose_name
            if not self._marker_server.get(pose_name):
                print 'creating marker'
                self.create_marker(pose_name)

    def processFeedback(self, msg):
        
        #### TODO: update position when the marker is moved!
        print 'feedback'
        interactive_marker = self._marker_server.get(msg.marker_name)
        user_action = UserAction()
        user_action.command = UserAction.EDIT
        user_action.pose_name = msg.marker_name
        user_action.pose = interactive_marker.pose
        self._pub.publish(user_action)



    def create_marker(self, pose_name):

        # Creates interactive marker with metadata, pose
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "odom"
        int_marker.name = pose_name
        int_marker.description = "Interactive Marker for Pose"

        int_marker.pose.position.x = 0
        int_marker.pose.position.y = 0
        int_marker.pose.position.z = 0
        int_marker.pose.orientation.w = 1

        # Create arrow and circle
        arrow_marker = Marker()
        arrow_marker.type = Marker.ARROW
        arrow_marker.scale.x = 0.5    # put it in place
        arrow_marker.scale.y = 0.5
        arrow_marker.scale.z = 0.5
        arrow_marker.color.r = 0.0    # make it pretty
        arrow_marker.color.g = 1.0
        arrow_marker.color.b = 0.0
        arrow_marker.color.a = 1.0

        # Create a non-interactive control which controls the arrow
        rotate_control = InteractiveMarkerControl()
        rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE
        rotate_control.always_visible = True
        rotate_control.orientation.w = 1
        rotate_control.orientation.x = 0;
        rotate_control.orientation.y = 1;
        rotate_control.orientation.z = 0;

        rotate_control.markers.append(arrow_marker)

        # Add the above control to the interactive marker
        int_marker.controls.append(rotate_control)

        # Apply changes to our marker server
        print 'inserting marker'
        print self._marker_server
        self._marker_server.insert(int_marker, self.processFeedback)
        self._marker_server.applyChanges()


def main():

    rospy.init_node('mmmm')

    # Initializing interactive markers
    marker_server = InteractiveMarkerServer("simple_server")

    monitor = InteractiveMarkerMonitor(marker_server)
    marker_server.applyChanges()
    rospy.spin()

if __name__ == '__main__':
    main()
