#!/usr/bin/env python
import pickle
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from moveit_msgs.msg import MoveItErrorCodes
import copy
import tf
import fetch_api
from robot_controllers_msgs.msg import QueryControllerStatesAction, QueryControllerStatesGoal, ControllerState
import actionlib
from visualization_msgs.msg import Marker
import copy
import tf.transformations as tft
from moveit_msgs.msg import OrientationConstraint
from moveit_python import PlanningSceneInterface
import numpy as np
from joint_state_reader import JointStateReader
from os.path import expanduser
from control_msgs.msg import FollowJointTrajectoryResult

PROGRAM_FILE = '{}/catkin_ws/src/cse481c/perception/nodes/programs.p'.format(expanduser('~'))
# TODO: we should sub to some other topic for the handle 
# pose. going through visualization marker is janky
SUB_NAME = '/visualization_marker'

ID_TO_TAGNAME = {'handle':'tray handle'}

class Program(object):
    def __init__(self, steps=None):
        if steps is None:
            steps = []
        self.steps = steps

    def __repr__(self):
        return "\n\n".join(list(map(str, (self.steps))))

    def __setstate__(self, d):
        self.__dict__ = d
        self.steps = copy.deepcopy(self.steps)

    def add_step(self, step, index=None):
        print 'index is {}'.format(index)
        if index is None:
            self.steps.append(step)
        else:
            self.steps.insert(index, step)

    def remove_step(self, index):
        del self.steps[index]


class ProgramStep(object):
    MOVE_ARM = "move_arm"
    MOVE_JOINT = "move_joint"
    MOVE_ALL_JOINTS = "move_all_joints"
    MOVE_TORSO = "move_torso"
    MOVE_GRIPPER = "move_gripper"
    #TODO: could add a gripper state
    def __init__(self, name = MOVE_ARM, pose=None, gripper_state=fetch_api.Gripper.OPENED, torso_height=0.4, has_constraint=False):
        self.step_type = name
        self.pose = pose
        self.gripper_state = gripper_state
        self.torso_height = torso_height
        self.has_constraint = has_constraint
        self.joint_name = ""
        self.joint_value = 0
        self.all_joint_states = []

    def __setstate__(self, d):
        if 'step_type' not in d:
            d['step_type'] = ProgramStep.MOVE_ARM
        self.__dict__ = d

    def __repr__(self):
        if self.step_type == ProgramStep.MOVE_ARM:
            p = self.pose.pose.position
            st_p = '({},{},{})'.format(p.x, p.y, p.z)
            o = self.pose.pose.orientation
            st_o = '({},{},{},{})'.format(o.x, o.y, o.z, o.w)
            return "\ttype: {}, {}: {} {}, gripper {}, torso height {}, has constraint {}".format(self.step_type, self.pose.header.frame_id, st_p, st_o, self.gripper_state, self.torso_height, self.has_constraint)
        elif self.step_type == ProgramStep.MOVE_JOINT:
            return "\ttype: {}, joint_name: {}, joint_value: {}".format(self.step_type, self.joint_name, self.joint_value)
        elif self.step_type == ProgramStep.MOVE_ALL_JOINTS:
            return "\ttype: {}, {}".format(self.step_type, dict(zip(fetch_api.ArmJoints.names(), self.all_joint_states)))
        elif self.step_type == ProgramStep.MOVE_TORSO:
            return "\ttype: {}, height: {}".format(self.step_type, self.torso_height)
        elif self.step_type == ProgramStep.MOVE_GRIPPER:
            return "\ttype: {}, gripper: {}".format(self.step_type, self.gripper_state)
        return "Not a valid step, {}".format(self.__dict__)

    def calc_pose(self, marker):
        frame_id = self.pose.header.frame_id
        if frame_id == "base_link":
            gripper_T_wrist = Pose(Point(-0.166, 0,0), Quaternion(0,0,0,1))
            new_pose = PoseStamped(pose=fetch_api.transform(self.pose.pose, gripper_T_wrist))
            new_pose.header.frame_id = 'base_link'
            return new_pose
        else:
            if frame_id not in ID_TO_TAGNAME:
                rospy.logerr("finding pose for a frame_id that isn't in ID_TO_TAGNAME")
                return None
            marker_id = ID_TO_TAGNAME[frame_id]
            if marker is None or marker_id != marker.ns:
                rospy.logerr("cannot find the {} with id {}".format(frame_id, marker_id))
                return None

            # we have to do some transformation magic
            # we have the base_link_T_tag frame from marker_match.pose
            # we have the tag_T_gripper from self.pose
            tag_T_gripper = copy.deepcopy(self.pose)
            #tag_T_wrist.pose.position.x -= 0.166
            base_link_T_tag = copy.deepcopy(marker)
            gripper_T_wrist = Pose(Point(-0.166, 0,0), Quaternion(0,0,0,1))
            base_link_T_gripper = fetch_api.transform(base_link_T_tag.pose, tag_T_gripper.pose)
            base_link_T_wrist = fetch_api.transform(base_link_T_gripper, gripper_T_wrist)
            new_pose = PoseStamped(pose=base_link_T_wrist)
            new_pose.header.frame_id = 'base_link'
            return new_pose

        return None 

class ProgramController(object):
    def __init__(self, program_file=PROGRAM_FILE):
        # TODO: Either implement behavior that fixes programs when markers change
        # or only let this callback run once
        self._markers_sub = rospy.Subscriber(SUB_NAME,
                                          Marker, 
                                          callback=self._markers_callback)
        self._curr_markers = None 
        self._tf_listener = tf.TransformListener()
        self._arm = fetch_api.Arm()
        self._gripper = fetch_api.Gripper()
        self._torso = fetch_api.Torso()
        self._controller_client = actionlib.SimpleActionClient('/query_controller_states', QueryControllerStatesAction)
        self._program_file = program_file
        self._programs = self._read_in_programs()

        self._joint_reader = JointStateReader()

        mat = tft.identity_matrix()
        mat[:,0] = np.array([0,0,-1,0])
        mat[:,2] = np.array([1,0,0,0])
        o = tft.quaternion_from_matrix(mat)
        self._constraint_pose = Pose(orientation=Quaternion(*o))

        oc = OrientationConstraint()
        oc.header.frame_id = 'base_link'
        oc.link_name = 'gripper_link'
        oc.orientation = self._constraint_pose.orientation
        oc.weight = 1.0
        oc.absolute_z_axis_tolerance = 1.0
        oc.absolute_x_axis_tolerance = 0.1
        oc.absolute_y_axis_tolerance = 1.0
        self._constraint = None
        

    def __str__(self):
        if self._programs:
            return "Programs:\n" + "\n".join(["{}:\n{}".format(name, program) for name, program in self._programs.items()])
        else:
            return "No programs"

    def _read_in_programs(self):
        try:
            with open(self._program_file, 'rb') as file:
                p = pickle.load(file)
                return p
        except IOError:
            print 'IOError on {}'.format(self._program_file)
            return {}

    def _write_out_programs(self):
        with open(self._program_file, 'wb') as file:
            pickle.dump(self._programs, file)

    def _markers_callback(self, msg):
        if msg.ns == "tray handle":
            self._curr_markers = msg 

    def remove_step(self, program_name, index):
        self._programs[program_name].remove_step(index)
        self._write_out_programs()


    def relax_arm(self):
        goal = QueryControllerStatesGoal()
        state = ControllerState()
        state.name = 'arm_controller/follow_joint_trajectory'
        state.state = ControllerState.STOPPED
        goal.updates.append(state)
        self._controller_client.send_goal(goal)
        self._controller_client.wait_for_result()

    def start_arm(self):
        goal = QueryControllerStatesGoal()
        state = ControllerState()
        state.name = 'arm_controller/follow_joint_trajectory'
        state.state = ControllerState.RUNNING
        goal.updates.append(state)
        self._controller_client.send_goal(goal)
        self._controller_client.wait_for_result()

    def close(self):
        self._gripper.close()

    def open(self):
        self._gripper.open()

    def save_all_joints(self, program_name, index=None):
        print "Saving all joints state as a program step"
        # need to grab the program as is
        curr_program = self._programs.get(program_name)
        if curr_program is None:
            print("{} does not exist yet".format(program_name))
            return

        step = ProgramStep()
        step.step_type = ProgramStep.MOVE_ALL_JOINTS
        step.all_joint_states = self._joint_reader.get_joints(fetch_api.ArmJoints.names())
        curr_program.add_step(step, index)
        self._write_out_programs()
    
    def add_constraint(self, program_name, index):
        print "adding constraint to step at index {} for program {}".format(index, program_name)
        curr_program = self._programs.get(program_name)
        if curr_program is None:
            print("{} does not exist yet".format(program_name))
            return

        curr_program.steps[index].has_constraint = True

    def save_joint(self, program_name, joint_name, index=None):
        print "Saving next joint state for program {} with name {}".format(program_name, joint_name)
        # need to grab the program as is
        curr_program = self._programs.get(program_name)
        if curr_program is None:
            print("{} does not exist yet".format(program_name))
            return

        step = ProgramStep()
        step.step_type = ProgramStep.MOVE_JOINT
        step.joint_name = joint_name

        # get current joint state value for joint name, if it exists
        joint_state = self._joint_reader.get_joints(fetch_api.ArmJoints.names())
        for i, name in enumerate(fetch_api.ArmJoints.names()):
            if step.joint_name == name:
                joint_value = joint_state[i]

        if joint_value is None:
            print "{} is not a value joint name. Try any of these {}".format(joint_name, fetch_api.ArmJoints.names())
            return

        step.joint_value = joint_value
        curr_program.add_step(step, index)
        self._write_out_programs()

    def add_torso(self, program_name, index=None):
        print "Saving next torso state for program {}".format(program_name)
        # need to grab the program as is
        curr_program = self._programs.get(program_name)
        if curr_program is None:
            print("{} does not exist yet".format(program_name))
            return

        step = ProgramStep()
        step.step_type = ProgramStep.MOVE_TORSO
        step.torso_height = self._torso.state()
        
        curr_program.add_step(step, index)
        self._write_out_programs()


    def save_gripper(self, program_name, index=None):
        print "Saving gripper state for program {}".format(program_name)
        # need to grab the program as is
        curr_program = self._programs.get(program_name)
        if curr_program is None:
            print("{} does not exist yet".format(program_name))
            return

        step = ProgramStep()
        step.step_type = ProgramStep.MOVE_GRIPPER
        step.gripper_state = self._gripper.state()
        curr_program.add_step(step, index)
        self._write_out_programs()

    # TODO: gripper status could be used here
    def save_program(self, program_name, frame_id, index=None, has_constraint=False):
        print "Saving next position for program {} in {}".format(program_name, frame_id)
        # need to grab the program as is
        curr_program = self._programs.get(program_name)
        if curr_program is None:
            print("{} does not exist yet".format(program_name))
            return

        # TODO: Complete this
        new_pose = PoseStamped()
        if frame_id in ID_TO_TAGNAME:
            # we know the user is trying to define a pose relative to a tag
            # we need to do some transformation stuff
            # need to grab the marker from self._curr_markers that matches frame_id
            marker_id = ID_TO_TAGNAME[frame_id]
            curr_marker = None
            if marker_id != self._curr_markers.ns:
                print('No marker found with frame_id {} and marker_id {} in program {}'.format(frame_id, marker_id, program_name))
                return
            else:
                # now we have the marker (curr_marker) which has some pose and we can get the robots current position for its arm
                # we have to then transform that current position to be relative to the curr_marker.pose

                # get position and orientation of the gripper in the base link 
                position, orientation = self._tf_listener.lookupTransform('/base_link', '/gripper_link', rospy.Time(0))
                base_link_T_wrist = Pose(Point(*position), Quaternion(*orientation))

                # The transformation from the base_link to this tags frame is just the tags pose
                base_link_T_tags = copy.deepcopy(self._curr_markers.pose)

                tags_T_wrist = fetch_api.transform(fetch_api.inverse_pose(base_link_T_tags), base_link_T_wrist)

                new_pose = PoseStamped(pose=tags_T_wrist)
                new_pose.header.frame_id = frame_id
        else:
            # user is trying to define frame_id in some arbitrary frame
            # assume base_link for now
            if frame_id == 'base_link':
                position, orientation = self._tf_listener.lookupTransform('/base_link', '/gripper_link', rospy.Time(0))
                base_link_T_wrist = Pose(Point(*position), Quaternion(*orientation))
                new_pose = PoseStamped(pose=base_link_T_wrist)
                new_pose.header.frame_id = frame_id
            else:
                print 'Please use base_link'
                return
        step = ProgramStep()
        step.pose = new_pose
        step.gripper_state = self._gripper.state()
        step.torso_height = self._torso.state()
        step.has_constraint = has_constraint
        curr_program.add_step(step, index)
        self._write_out_programs()

    def rename_program(self, program_name, new_program_name):
        if program_name not in self._programs:
            print '{} does not exist'.format(program_name)
            return
        if new_program_name in self._programs:
            print '{} already exists'.format(new_program_name)
            return

        self._programs[new_program_name] = self._programs[program_name]
        del self._programs[program_name]

    def deque_step(self, program_name):
        if program_name not in self._programs:
            print '{} does not exist'.format(program_name)
            return
        self._programs[program_name].remove_step(len(self._programs[program_name].steps) - 1)
        self._write_out_programs()

    def create_program(self, program_name):
        if program_name not in self._programs:
            self._programs[program_name] = Program()
            print "{} created".format(program_name)
            print "the program is {}".format(self._programs[program_name])
        else:
            print "Trying to create program {} when it already exists".format(program_name)
        self._write_out_programs()

    def delete_program(self, program_name):
        if program_name in self._programs:
            del self._programs[program_name]
            print "{} deleted".format(program_name)
            self._write_out_programs()
        else:
            print "{} does not exist".format(program_name)

    def run_program(self, program_name):
        if program_name not in self._programs:
            print "{} does not exist".format(program_name)
            return False
        else:
            self.start_arm()
            curr_marker = copy.deepcopy(self._curr_markers)
            print 'PRINTING CURR MARKER'
            print curr_marker
            for i, cur_step in enumerate(self._programs[program_name].steps):
                if cur_step.step_type == ProgramStep.MOVE_ARM:
                    # if cur_step.gripper_state != self._gripper.state():
                    #     if cur_step.gripper_state == fetch_api.Gripper.OPENED:
                    #         self._gripper.open(max_effort=0.7)
                    #     else:
                    #         self._gripper.close(max_effort=0.7)
                    #     # don't move the arm if we move the gripper
                    #     print 'adjust gripper successful'
                    #     continue

                    pose = cur_step.calc_pose(curr_marker)
                    if pose is None:
                        return False
                    if cur_step.has_constraint:
                        error = self._arm.move_to_pose(pose, orientation_constraint=self._constraint, allowed_planning_time=25.0, num_planning_attempts=3, replan=True)
                    else:
                        error = self._arm.move_to_pose(pose, allowed_planning_time=25.0, num_planning_attempts=3, replan=True)
                    
                    if error is not None:
                        print "move arm failed with error {}".format(error)
                        print "{} failed to run at step #{}".format(program_name, i+1)
                        return False
                    else:
                        print 'move arm successful'

                # moving a joint
                if cur_step.step_type == ProgramStep.MOVE_JOINT:
                    # grab the current joint state and update the value from
                    # this program step
                    joint_state = self._joint_reader.get_joints(fetch_api.ArmJoints.names())
                    for i, name in enumerate(fetch_api.ArmJoints.names()):
                        if cur_step.joint_name == name:
                            joint_state[i] = cur_step.joint_value

                    # grab an arm joint object from this list of joint positions
                    arm_joints = fetch_api.ArmJoints.from_list(joint_state)
                    res = self._arm.move_to_joints(arm_joints)
                    if res.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
                        print 'joint adjustment failed with error code {} and error string {}'.format(res.error_code, res.error_string)
                        print "{} failed to run at step #{}".format(program_name, i+1)
                        return False
                    else:
                        print 'joint adjustment successful'

                # moving all joints
                if cur_step.step_type == ProgramStep.MOVE_ALL_JOINTS:
                    arm_joints = fetch_api.ArmJoints.from_list(cur_step.all_joint_states)
                    res = self._arm.move_to_joints(arm_joints)
                    if res.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
                        print 'all joints adjustment failed with error code {} and error string {}'.format(res.error_code, res.error_string)
                        print "{} failed to run at step #{}".format(program_name, i+1)
                        return False
                    else:
                        print 'all joints adjustment successful'


                # move torso
                if cur_step.step_type == ProgramStep.MOVE_TORSO:
                    res = self._torso.set_height(cur_step.torso_height)
                    if res.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
                        print 'torso adjustment failed with error code {} and error string {}'.format(res.error_code, res.error_string)
                        print "{} failed to run at step #{}".format(program_name, i+1)
                        return False
                    else:
                        print 'torso adjustment successful'

                if cur_step.step_type == ProgramStep.MOVE_GRIPPER:
                    if cur_step.gripper_state == fetch_api.Gripper.OPENED:
                        self._gripper.open(max_effort=75)
                    else:
                        self._gripper.close(max_effort=75)
                    print 'gripper adjustment successful'
                    rospy.sleep(1.5)
            return True




    @property
    def programs(self):
        return self._programs
