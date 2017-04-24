#!/usr/bin/env python

import rospy
from pose_ctrl import PoseController
from marker_ctrl import MarkerController
from map_annotator.msg import UserAction, PoseNames
from interactive_markers.interactive_marker_server import InteractiveMarkerServer

SUB_NAME = "/user_actions"
PUB_NAME = "/pose_names"

class MapAnnotatorMonitor(object):
    def __init__(self, pose_ctrl, marker_ctrl):
        self._pose_ctrl = pose_ctrl
        self._marker_ctrl = marker_ctrl
        self._sub = rospy.Subscriber(SUB_NAME, UserAction, self._callback)
        self._pub = rospy.Publisher(PUB_NAME, PoseNames, queue_size=10, latch=True)
        self._publish_pose_names()

    def _publish_pose_names(self):
        pose_names = PoseNames()
        pose_names.names = [key for key in self._pose_ctrl.poses]
        self._pub.publish(pose_names)

    def _callback(self, msg):
        if msg.command == UserAction.SAVE:
            self._pose_ctrl.save_pose(msg.pose_name)
            self._marker_ctrl.create_marker(msg.pose_name, self._pose_ctrl.curr_pose.pose.pose)
            self._publish_pose_names()
        elif msg.command == UserAction.DELETE:
            self._pose_ctrl.delete_pose(msg.pose_name)
            self._marker_ctrl.erase_marker(msg.pose_name)
            self._publish_pose_names()
        elif msg.command == UserAction.RENAME:
            self._pose_ctrl.rename_pose(msg.pose_name, msg.pose_name_new)
            self._publish_pose_names()
        elif msg.command == UserAction.GOTO:
            self._pose_ctrl.move_to_pose(msg.pose_name)
        else:
            print("Invalid command")

def main():
    rospy.init_node('map_annotator')
    server = InteractiveMarkerServer("simple_server")
    pose_ctrl = PoseController()
    marker_ctrl = MarkerController(server, pose_ctrl)
    monitor = MapAnnotatorMonitor(pose_ctrl, marker_ctrl)
    rospy.spin()

if __name__ == '__main__':
    main()
