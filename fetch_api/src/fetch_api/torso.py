#!/usr/bin/env python

import actionlib
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
import rospy

ACTION_NAME = 'torso_controller/follow_joint_trajectory'
JOINT_NAME = 'torso_lift_joint'
TIME_FROM_START = 5  # How many seconds it should take to set the torso height.


class Torso(object):
    """Torso controls the robot's torso height.
    """
    MIN_HEIGHT = 0.0
    MAX_HEIGHT = 0.4

    def __init__(self):
        # TODO: Create actionlib client
        # TODO: Wait for server
        self.client = actionlib.SimpleActionClient(ACTION_NAME, FollowJointTrajectoryAction)
        self.client.wait_for_server()

    def set_height(self, height):
        """Sets the torso height.

        This will always take ~5 seconds to execute.

        Args:
            height: The height, in meters, to set the torso to. Values range
                from Torso.MIN_HEIGHT (0.0) to Torso.MAX_HEIGHT(0.4).
        """
        # TODO: Check that the height is between MIN_HEIGHT and MAX_HEIGHT.
        # TODO: Create a trajectory point
        # TODO: Set position of trajectory point
        # TODO: Set time of trajectory point
        if not Torso.MIN_HEIGHT <= height <= Torso.MAX_HEIGHT:
            return

        point = JointTrajectoryPoint()
        point.positions.append(height)
        point.time_from_start = rospy.Duration(TIME_FROM_START)

        # TODO: Create goal
        # TODO: Add joint name to list
        # TODO: Add the trajectory point created above to trajectory
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names.append(JOINT_NAME)
        goal.trajectory.points.append(point)

        # TODO: Send goal
        # TODO: Wait for result
        self.client.send_goal(goal)
        self.client.wait_for_result()
        print(self.client.get_result())

