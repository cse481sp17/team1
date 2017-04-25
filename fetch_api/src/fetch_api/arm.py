import rospy
import actionlib
from .arm_joints import ArmJoints
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from .moveit_goal_builder import MoveItGoalBuilder
from moveit_msgs.msg import MoveItErrorCodes, MoveGroupAction   
from geometry_msgs.msg import Pose

ACTION_NAME_ARM_CONTROL = 'arm_controller/follow_joint_trajectory'
ACTION_NAME_MOVE_GROUP = 'move_group'
TIME_FROM_START = 5

class Arm(object):
    """Arm controls the robot's arm.

    Joint space control:
        joints = ArmJoints()
        # Fill out joint states
        arm = fetch_api.Arm()
        arm.move_to_joints(joints)
    """

    def __init__(self):
        # TODO: Create actionlib client
        self.arm_client = actionlib.SimpleActionClient(ACTION_NAME_ARM_CONTROL, FollowJointTrajectoryAction)

        # Creates a MoveGroupAction client
        self.move_group_client = actionlib.SimpleActionClient(ACTION_NAME_MOVE_GROUP, MoveGroupAction)

        # TODO: Wait for server
        self.arm_client.wait_for_server()
        self.move_group_client.wait_for_server()

    def move_to_joints(self, arm_joints):
        """Moves the robot's arm to the given joints.

        Args:
            arm_joints: An ArmJoints object that specifies the joint values for
                the arm.
        """
        # TODO: Create a trajectory point
    	point = JointTrajectoryPoint()

            # TODO: Set position of trajectory point
    	for joint in arm_joints.values():
	        point.positions.append(joint)

        # TODO: Set time of trajectory point
        point.time_from_start = rospy.Duration(TIME_FROM_START)

        # TODO: Create goal
        # TODO: Add joint name to list
        # TODO: Add the trajectory point created above to trajectory
        goal = FollowJointTrajectoryGoal()
        for joint_name in arm_joints.names():
	        goal.trajectory.joint_names.append(joint_name)
        goal.trajectory.points.append(point)

        # TODO: Send goal
        # TODO: Wait for result
        self.arm_client.send_goal(goal)
        self.arm_client.wait_for_result()
        print(self.arm_client.get_result())


    def moveit_error_string(self, val):
        """Returns a string associated with a MoveItErrorCode.
            
        Args:
            val: The val field from moveit_msgs/MoveItErrorCodes.msg
            
        Returns: The string associated with the error value, 'UNKNOWN_ERROR_CODE'
            if the value is invalid.
        """ 
        if val == MoveItErrorCodes.SUCCESS:
            return 'SUCCESS'
        elif val == MoveItErrorCodes.FAILURE:
            return 'FAILURE'
        elif val == MoveItErrorCodes.PLANNING_FAILED:
            return 'PLANNING_FAILED'
        elif val == MoveItErrorCodes.INVALID_MOTION_PLAN:
            return 'INVALID_MOTION_PLAN'
        elif val == MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
            return 'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE'
        elif val == MoveItErrorCodes.CONTROL_FAILED:
            return 'CONTROL_FAILED'
        elif val == MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA:
            return 'UNABLE_TO_AQUIRE_SENSOR_DATA'
        elif val == MoveItErrorCodes.TIMED_OUT:
            return 'TIMED_OUT'
        elif val == MoveItErrorCodes.PREEMPTED:
            return 'PREEMPTED'
        elif val == MoveItErrorCodes.START_STATE_IN_COLLISION:
            return 'START_STATE_IN_COLLISION'
        elif val == MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS:
            return 'START_STATE_VIOLATES_PATH_CONSTRAINTS'
        elif val == MoveItErrorCodes.GOAL_IN_COLLISION:
            return 'GOAL_IN_COLLISION'
        elif val == MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS:
            return 'GOAL_VIOLATES_PATH_CONSTRAINTS'
        elif val == MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED:
            return 'GOAL_CONSTRAINTS_VIOLATED'
        elif val == MoveItErrorCodes.INVALID_GROUP_NAME:
            return 'INVALID_GROUP_NAME'
        elif val == MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS:
            return 'INVALID_GOAL_CONSTRAINTS'
        elif val == MoveItErrorCodes.INVALID_ROBOT_STATE:
            return 'INVALID_ROBOT_STATE'
        elif val == MoveItErrorCodes.INVALID_LINK_NAME:
            return 'INVALID_LINK_NAME'                                      
        elif val == MoveItErrorCodes.INVALID_OBJECT_NAME:
            return 'INVALID_OBJECT_NAME'
        elif val == MoveItErrorCodes.FRAME_TRANSFORM_FAILURE:
            return 'FRAME_TRANSFORM_FAILURE'
        elif val == MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE:
            return 'COLLISION_CHECKING_UNAVAILABLE'
        elif val == MoveItErrorCodes.ROBOT_STATE_STALE:
            return 'ROBOT_STATE_STALE'
        elif val == MoveItErrorCodes.SENSOR_INFO_STALE:
            return 'SENSOR_INFO_STALE'
        elif val == MoveItErrorCodes.NO_IK_SOLUTION:
            return 'NO_IK_SOLUTION'
        else:
            return 'UNKNOWN_ERROR_CODE'


    def move_to_pose(self, pose_stamped):
        """
        Moves the end-effector to a pose, using motion planning.

        Args:
            pose: geometry_msgs/PoseStamped. The goal pose for the gripper.

        Returns:
            string describing the error if an error occurred, else None.
        """

        # Creates a goal
        goal_builder = MoveItGoalBuilder()
        goal_builder.set_pose_goal(pose_stamped)
        goal = goal_builder.build()

        # Send goal
        self.move_group_client.send_goal(goal)

        # Set a timeout and wait for result
        self.move_group_client.wait_for_result(rospy.Duration(10))
        self.move_group_client.wait_for_result()

        result = self.move_group_client.get_result()
        if result:
            print self.moveit_error_string(result.error_code.val)
        else:   # success
            return None


    def cancel_all_goals(self):
        """
        For safety reasons!
        """
        self.arm_client.cancel_all_goals() # Your action client from Lab 7
        self.move_group_client.cancel_all_goals() # From this lab

