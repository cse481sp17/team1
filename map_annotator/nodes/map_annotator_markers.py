import rospy
from map_annotator_node import MapAnnotatorMonitor
from map_annotator_ctrl import PoseController
from map_annotator.msg import UserAction
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker

SUB_NAME = "/user_actions"
PUB_NAME = "/pose_names"

class MapAnnotatorMonitor(object):
    def __init__(self, ctrl):
        self._ctrl = ctrl
        self._sub = rospy.Subscriber(SUB_NAME, UserAction, self._callback)
        self._pub = rospy.Publisher(PUB_NAME, PoseNames, queue_size=10, latch=True)
        self._publish_pose_names()

    def _publish_pose_names(self):
        pose_names = PoseNames()
        pose_names.names = list(self._ctrl.poses.keys())
        self._pub.publish(pose_names)

    def _callback(self, msg):
        global marker_Server

        if msg.command == UserAction.SAVE and msg.pose_name:
            self._ctrl.save_pose(msg.pose_name)
            create_marker(msg.pose_name, ctrl.curr_pose)
            self._publish_pose_names()
        elif msg.command == UserAction.DELETE and msg.pose_name:
            self._ctrl.delete_pose(msg.pose_name)
            marker_server.erase(msg.pose_name)
            self._publish_pose_names()
        elif msg.command == UserAction.RENAME and msg.pose_name and msg.pose_name_new:
            self._ctrl.rename_pose(msg.pose_name, msg.pose_name_new)
            self._publish_pose_names()
        elif msg.command == UserAction.GOTO and msg.pose_name:
            self._ctrl.move_to_pose(msg.pose_name)
        else:
            print("Invalid command")


def processFeedback(feedback):

    #### TODO: update position when the marker is moved!
    

    p = feedback.pose.position
    print feedback.marker_name + " is now at" + str(p.x) + ", " + str(p.y) + ", " + str(p.z)


def create_marker(pose_name, pose):
    global marker_server

    # Creates interactive marker with metadata, pose
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "map"
    int_marker.name = pose_name
    int_marker.description = "Interactive Marker for Pose"
    int_marker.pose = pose

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
    marker_server.insert(int_marker, processFeedback)
    marker_server.applyChanges()


def main():
    global marker_server

    rospy.init_node('map_annotator')
    ctrl = PoseController()
    monitor = MapAnnotatorMonitor(ctrl)
    
    # Initializing interactive markers
    marker_server = InteractiveMarkerServer("simple_marker")

    # Create an interactive marker for each pose in our list
    for pose_name in ctrl.poses:
        create_marker(pose_name, poses[pose_name])

    rospy.spin()

if __name__ == '__main__':
    main()

if __name__ == '__main__':
    main()
