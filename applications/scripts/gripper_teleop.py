#!/usr/bin/env python
import rospy, copy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import *
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from std_msgs.msg import ColorRGBA
import fetch_api
from gripper_teleop_util import *

class GripperTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server

    def start(self):
        pose = Pose(orientation=Quaternion(0,0,0,1))
        gripper_im = create_gripper_marker(pose)

        self._im_server.insert(gripper_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()

    def handle_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.menu_entry_id == GRIPPER_POSE_ID:
                ps = PoseStamped()
                ps.header.frame_id = 'base_link'
                ps.header.stamp = rospy.get_rostime()
                ps.pose = feedback.pose
                self._arm.move_to_pose(ps) # maybe loop if there is an error
            elif feedback.menu_entry_id == OPEN_GRIPPER_ID:
                self._gripper.open()
            elif feedback.menu_entry_id == CLOSE_GRIPPER_ID:
                self._gripper.close()
        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            ps = PoseStamped()
            ps.pose = feedback.pose
            ps.header.frame_id = 'base_link'
            gripper_im = self._im_server.get(feedback.marker_name)
            possible = self._arm.compute_ik(ps)
            for marker in gripper_im.controls[0].markers:
                if possible:
                    marker.color = GREEN
                else: 
                    marker.color = RED
            self._im_server.erase(feedback.marker_name)
            self._im_server.insert(gripper_im)
            self._im_server.applyChanges()


       

class AutoPickTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server

    def start(self):
        # obj_im = InteractiveMarker() ...
        #self._im_server.insert(obj_im, feedback_cb=self.handle_feedback)
        pass

    def handle_feedback(self, feedback):
        pass


def main():
    #
    #
    # Initialize the interactive marker server for the gripper
    rospy.init_node('gripper_demo')
    im_server = InteractiveMarkerServer('gripper_im_server', q_size=2)
    auto_pick_im_server = InteractiveMarkerServer('auto_pick_im_server', q_size=2)
    arm = fetch_api.Arm()
    gripper = fetch_api.Gripper()

    teleop = GripperTeleop(arm, gripper, im_server)
    #auto_pick = AutoPickTeleop(arm, gripper, auto_pick_im_server)
    teleop.start()
    # auto_pick.start()
    rospy.spin()

if __name__ == '__main__':
    main()
