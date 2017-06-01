#!/usr/bin/env python
from navigator_msg.srv import Navigation, NavigationRequest
import rospy
from pose_ctrl import PoseController
import fetch_api

class Navigator:
    def __init__(self):
        self._pose_ctrl = PoseController()
        self._head = fetch_api.Head()

    # TODO: program the navigation with cli.py
    def navigate(self, req):
        print req
        return self.run_procedure(req.location)

    def run_procedure(self, name):
        if name not in self._pose_ctrl._poses:
            print("{} program does not exist".format(name))
            return False

        # set the head to the ceiling
        self._head.pan_tilt(0, fetch_api.Head.MIN_TILT)

        return self._pose_ctrl.move_to_pose(name)


if __name__ == '__main__':
    rospy.init_node("navigation")
    navigator = Navigator()
    s = rospy.Service('navigation', Navigation, navigator.navigate)
    rospy.spin()