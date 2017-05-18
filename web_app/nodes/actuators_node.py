#!/usr/bin/env python

import fetch_api
import rospy
from web_teleop.srv import *

CURRENT_POSE = 0
DISCO_POSES = [[1.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0],
               [0.8, 0.75, 0.0, -2.0, 0.0, 2.0, 0.0],
               [-0.8, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0],
               [-1.5, 1.1, -3.0, -0.5, -3.0, -1.0, -3.0],
               [-0.8, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0],
               [0.8, 0.75, 0.0, -2.0, 0.0, 2.0, 0.0],
               [1.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0]]
SET_STRAIGHT_ARM = [-0.522, -1.124, 0.856, 1.235, 2.788, -1.590, -1.402]

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

class ActuatorServer(object):
    def __init__(self):
        self._torso = fetch_api.Torso()
        self._gripper = fetch_api.Gripper()
        self._head = fetch_api.Head()
        self._arm = fetch_api.Arm()
        self._current_pose = 0

    def handle_set_torso(self, request):
        # TODO: move the torso to the requested height
        self._torso.set_height(request.height)
        return SetTorsoResponse()

    def handle_set_gripper(self, request):
        self._gripper.open() if request.open else self._gripper.close()
        return SetGripperResponse()

    def handle_set_head_look_at(self, request):
        self._head.look_at(request.frame_id,
                           request.x,
                           request.y,
                           request.z)
        return SetHeadLookAtResponse()

    def handle_set_head_pan_tilt(self, request):
        self._head.pan_tilt(request.pan, request.tilt)
        return SetHeadPanTiltResponse()

    def handle_set_arm_strike_pose(self, request):
        # joints = DISCO_POSES[self._current_pose]
	joints = SET_STRAIGHT_ARM        
	# self._current_pose = (self._current_pose + 1) % len(DISCO_POSES)
        arm_joints = fetch_api.ArmJoints.from_list(joints)
        self._arm.move_to_joints(arm_joints)
        return SetArmStrikePoseResponse()

def main():
    rospy.init_node('web_teleop_actuators')
    wait_for_time()
    server = ActuatorServer()
    torso_service = rospy.Service('web_teleop/set_torso', SetTorso,
                                  server.handle_set_torso)
    gripper_service = rospy.Service('web_teleop/set_gripper', SetGripper,
                                    server.handle_set_gripper)
    head_look_at_service = rospy.Service('web_teleop/set_head_look_at', SetHeadLookAt,
                                         server.handle_set_head_look_at)
    head_pan_tilt_service = rospy.Service('web_teleop/set_head_pan_tilt', SetHeadPanTilt,
                                          server.handle_set_head_pan_tilt)
    arm_strike_pose_service = rospy.Service('web_teleop/set_arm_strike_pose', SetArmStrikePose,
                                server.handle_set_arm_strike_pose)
    rospy.spin()


if __name__ == '__main__':
    main()

