#!/usr/bin/env python

import rospy
from map_annotator_ctrl import PoseController
from map_annotator.msg import UserAction, PoseNames

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
        if msg.command == UserAction.SAVE and msg.pose_name:
            self._ctrl.save_pose(msg.pose_name)
            self._publish_pose_names()
        elif msg.command == UserAction.DELETE and msg.pose_name:
            self._ctrl.delete_pose(msg.pose_name)
            self._publish_pose_names()
        elif msg.command == UserAction.RENAME and msg.pose_name and msg.pose_name_new:
            self._ctrl.rename_pose(msg.pose_name, msg.pose_name_new)
            self._publish_pose_names()
        elif msg.command == UserAction.GOTO and msg.pose_name:
            self._ctrl.move_to_pose(msg.pose_name)
        else:
            print("Invalid command")

def main():
    rospy.init_node('map_annotator')
    ctrl = PoseController()
    monitor = MapAnnotatorMonitor(ctrl)
    rospy.spin()

if __name__ == '__main__':
    main()
