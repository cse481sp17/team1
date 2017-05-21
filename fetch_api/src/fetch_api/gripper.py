#! /usr/bin/env python
import actionlib
import control_msgs.msg
from sensor_msgs.msg import JointState
import rospy

# TODO: ACTION_NAME = ???
CLOSED_POS = 0.0  # The position for a fully-closed gripper (meters).
OPENED_POS = 0.10  # The position for a fully-open gripper (meters).

R_NAME = 'r_gripper_finger_joint'
L_NAME = 'l_gripper_finger_joint'

class Gripper(object):
    """Gripper controls the robot's gripper.
    """
    MIN_EFFORT = 35  # Min grasp force, in Newtons
    MAX_EFFORT = 100  # Max grasp force, in Newtons
    CLOSED = 1
    OPENED = 0
    def __init__(self):
        self.client = actionlib.SimpleActionClient('gripper_controller/gripper_action', control_msgs.msg.GripperCommandAction)
        self.client.wait_for_server()
        self.joint_sub = rospy.Subscriber('/joint_states', JointState, self._callback)
        self.r_finger_position = None
        self.l_finger_position = None

    def _callback(self, msg):
        if r_name in msg.name:
            self.r_finger_position = msg.position[msg.name.index(R_NAME)]
        if l_name in msg.name:
            self.l_finger_position = msg.position[msg.name.index(L_NAME)]

    def open(self):
        """Opens the gripper.
        """
        # TODO: Create goal
        # TODO: Send goal
        # TODO: Wait for result
        goal = control_msgs.msg.GripperCommandGoal()
        goal.command.position = OPENED_POS
        goal.command.max_effort = Gripper.MAX_EFFORT
        self.client.send_goal(goal)
        self.client.wait_for_result()


    def close(self, max_effort=MAX_EFFORT):
        """Closes the gripper.

        Args:
            max_effort: The maximum effort, in Newtons, to use. Note that this
                should not be less than 35N, or else the gripper may not close.
        """
        # TODO: Create goal
        # TODO: Send goal
        # TODO: Wait for result
        goal = control_msgs.msg.GripperCommandGoal()
        goal.command.position = CLOSED_POS
        goal.command.max_effort = max_effort
        self.client.send_goal(goal)
        self.client.wait_for_result()

    def state(self):
        val = round(self.r_finger_position, 2)
        if val == 0.05:
            return Gripper.OPENED
        return Gripper.CLOSED

