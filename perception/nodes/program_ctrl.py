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

PROGRAM_FILE = '/home/team1/catkin_ws/src/cse481c/perception/nodes/programs.p'
# TODO: we should sub to some other topic for the handle 
# pose. going through visualization marker is janky
SUB_NAME = '/visualization_marker'

ID_TO_TAGNAME = {'handle':400}

class Program(object):
    def __init__(self, steps=[]):
        self.steps = steps

    def __repr__(self):
        # ret = "["
        # first = True
        # for step in self.steps:
        #     if first:
        #         ret += step.__str__()
        #         first = False
        #     else:
        #         ret += ", {}".format(step.__str__())
        # ret += "]"
        # return self.steps.__str__()
        return "\n\n".join(list(map(str, (self.steps))))


    def add_step(self, step, append=True):
        if append:
            self.steps.append(step)
        else:
            self.steps.insert(0, step)

    def remove_step(self, index):
        del self.steps[index]

    def calc_poses(self, markers):
        ret = []
        for step in self.steps:
            pose = self._find_pose(step, markers)
            if pose is None:
                print 'cannot run program that uses markers when there are no markers'
                return []
            ret.append(pose)
        return ret

    def _find_pose(self, step, markers):
        frame_id = step.pose.header.frame_id
        if frame_id == "base_link":
            gripper_T_wrist = Pose(Point(-0.166, 0,0), Quaternion(0,0,0,1))
            new_pose = PoseStamped(pose=fetch_api.transform(step.pose.pose, gripper_T_wrist))
            new_pose.header.frame_id = 'base_link'
            return new_pose
        else:
            if frame_id not in ID_TO_TAGNAME:
                rospy.logerr("finding pose for a frame_id that isn't in ID_TO_TAGNAME")
                return None
            marker_id = ID_TO_TAGNAME[frame_id]

            if marker_id != markers.id:
                rospy.logerr("cannot find the {} with id {}".format(frame_id, marker_id))
                return None

            # we have to do some transformation magic
            # we have the base_link_T_tag frame from marker_match.pose
            # we have the tag_T_gripper from step.pose
            tag_T_gripper = copy.deepcopy(step.pose)
            #tag_T_wrist.pose.position.x -= 0.166
            base_link_T_tag = copy.deepcopy(markers)
            gripper_T_wrist = Pose(Point(-0.166, 0,0), Quaternion(0,0,0,1))
            base_link_T_gripper = fetch_api.transform(base_link_T_tag.pose, tag_T_gripper.pose)
            base_link_T_wrist = fetch_api.transform(base_link_T_gripper, gripper_T_wrist)
            new_pose = PoseStamped(pose=base_link_T_wrist)
            new_pose.header.frame_id = 'base_link'
            return new_pose

        return None 

class ProgramStep(object):
    #TODO: could add a gripper state
    def __init__(self, pose=None, gripper_state=fetch_api.Gripper.OPENED, torso_height=0.4):
        self.pose = pose
        self.gripper_state = gripper_state
        self.torso_height = torso_height

    def __repr__(self):
        if self.pose is None:
            return "Empty"
        p = self.pose.pose.position
        st_p = '({},{},{})'.format(p.x, p.y, p.z)
        o = self.pose.pose.orientation
        st_o = '({},{},{},{})'.format(o.x, o.y, o.z, o.w)
        return "\t{}: {} {}, gripper {}, torso height {}".format(self.pose.header.frame_id, st_p, st_o, self.gripper_state, self.torso_height)


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

    def __str__(self):
        if self._programs:
            return "Programs:\n" "\n".join(["{}:\n{}".format(name, program) for name, program in self._programs.items()])
        else:
            return "No programs"

    def _read_in_programs(self):
        try:
            with open(self._program_file, 'rb') as file:
                return pickle.load(file)
        except IOError:
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

    # TODO: gripper status could be used here
    def save_program(self, program_name, frame_id, append=True):
        print "Saving next position for program {} in {}".format(program_name, frame_id)
        # need to grab the program as is
        curr_program = self._programs.get(program_name)
        if curr_program is None:
            rospy.logerr("This shouldn't happen if create has been called")
            return

        # TODO: Complete this
        new_pose = PoseStamped()
        if frame_id in ID_TO_TAGNAME:
            # we know the user is trying to define a pose relative to a tag
            # we need to do some transformation stuff
            # need to grab the marker from self._curr_markers that matches frame_id
            marker_id = ID_TO_TAGNAME[frame_id]
            curr_marker = None
            if marker_id != self._curr_markers.id:
                rospy.logerr('No marker found with frame_id {} and marker_id {} in program {}'.format(frame_id, marker_id, program_name))
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
        step = ProgramStep(new_pose)
        step.gripper_state = self._gripper.state()
        step.torso_height = self._torso.state()
        curr_program.add_step(step, append)
        self._write_out_programs()

    def create_program(self, program_name):
        if program_name not in self._programs:
            self._programs[program_name] = Program()
            print "{} created".format(program_name)
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
        else:      
            # only move arm to pose if we aren't changing height
            doPose = True
            poses = self._programs[program_name].calc_poses(self._curr_markers)
            self.start_arm()

            prevHeight = None
            for i, pose in enumerate(poses):
                height = self._programs[program_name].steps[i].torso_height
                self._torso.set_height(height)
                if prevHeight != None and prevHeight != height:
                    doPose = False
                
                prevHeight = height

                if self._programs[program_name].steps[i].gripper_state == fetch_api.Gripper.OPENED:
                    self._gripper.open()
                else:
                    self._gripper.close()

                if doPose:
                    error = self._arm.move_to_pose(pose, allowed_planning_time=15.0)
                    if error is not None:
                        print "{} failed to run at step #{}".format(program_name, i+1)
                        return
                rospy.sleep(1.5)


    @property
    def programs(self):
        return self._programs
