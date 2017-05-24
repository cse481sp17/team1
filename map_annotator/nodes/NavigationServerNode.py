#!/usr/bin/env python
from navigator_msg.srv import Navigation, NavigationRequest
import rospy
from pose_ctrl import PoseController

PROGRAM_FILE = "programs.p"

class Navigator:
    def __init__(self):
        self._pose_ctrl = PoseController(PROGRAM_FILE)

    def navigate(self, req):
        print req
        return self.run_procedure(req.location)
        return False

    def run_procedure(self, name):
        if name not in self._pose_ctrl._poses:
            print("{} program does not exist".format(name))
            return False
        #TODO have run_program return a boolean
        # false if the program could not run
        self._pose_ctrl.move_to_pose(name)
        return True


if __name__ == '__main__':
    rospy.init_node("navigation")
    navigator = Navigator()
    s = rospy.Service('navigation', Navigation, navigator.navigate)
    rospy.spin()