import pickle
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from ar_track_alvar_msgs.msg import AlvarMarkers
from moveit_msgs.msg import MoveItErrorCodes
import copy
import tf
import fetch_api

POSE_FILE = '/home/team1/catkin_ws/src/cse481c/ar_pdb/nodes/programs.p'
SUB_NAME = '/ar_pose_marker'

ID_TO_TAGNAME = {'tag1':15, 'tag2':2}

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
        return self.steps.__repr__()


    def add_step(self, step):
        self.steps.append(step)

    def calc_poses(self, markers):
        ret = []
        for step in self.steps:
            ret.append(self._find_pose(step, markers))
        return ret

    def _find_pose(self, step, markers):
        frame_id = step.pose.header.frame_id
        if frame_id == "base_link":
            return step.pose
        else:
            if frame_id not in ID_TO_TAGNAME:
                rospy.logerr("finding pose for a frame_id that isn't in ID_TO_TAGNAME")
                return None
            marker_id = ID_TO_TAGNAME[frame_id]
            marker_match = None
            for marker in markers:
                if marker_id == marker.id:
                    marker_match = marker
                    break

            if marker_match is None:
                rospy.logerr("cannot find the {} with id {}".format(frame_id, marker_id))
                return

            # we have to do some transformation magic
            # we have the base_link_T_tag frame from marker_match.pose
            # we have the tag_T_gripper from step.pose
            tag_T_gripper = copy.deepcopy(step.pose)
            base_link_T_tag = copy.deepcopy(marker_match.pose)
            base_link_T_gripper = fetch_api.transform(base_link_T_tag.pose, tag_T_gripper.pose)
            new_pose = PoseStamped(pose=base_link_T_gripper)
            new_pose.header.frame_id = 'base_link'
            return new_pose

        return None 

class ProgramStep(object):
    #TODO: could add a gripper state
    def __init__(self, pose=None):
        self.pose = pose

    def __repr__(self):
        if self.pose is None:
            return "Empty"
        return self.pose.__str__()


class ProgramController(object):
    def __init__(self):
        # TODO: Either implement behavior that fixes programs when markers change
        # or only let this callback run once
        self._markers_sub = rospy.Subscriber(SUB_NAME,
                                          AlvarMarkers, 
                                          callback=self._markers_callback)
        self._programs = self._read_in_programs()
        self._curr_markers = None
        self._tf_listener = tf.TransformListener()
        self._arm = fetch_api.Arm()
        rospy.sleep(0.1)

    def __str__(self):
        if self._programs:
            return "Programs:\n" "\n".join(["{}:\n{}".format(name, pose) for name, pose in self._programs.items()])
        else:
            return "No programs"

    def _read_in_programs(self):
        try:
            with open(POSE_FILE, 'rb') as file:
                return pickle.load(file)
        except IOError:
            return {}

    def _write_out_programs(self):
        with open(POSE_FILE, 'wb') as file:
            pickle.dump(self._programs, file)

    def _markers_callback(self, msg):
        self._curr_markers = msg.markers

    # TODO: gripper status could be used here
    def save_program(self, program_name, frame_id):
        if not self._curr_markers:
            print "No ar markers available"
            return
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
            for marker in self._curr_markers:
                if marker.id == marker_id:
                    curr_marker = marker
                    break
            if curr_marker is None:
                rospy.logerr('No marker found with frame_id {} and marker_id {} in program {}'.format(frame_id, marker_id, program_name))
                return
            else:
                # now we have the marker (curr_marker) which has some pose and we can get the robots current position for its arm
                # we have to then transform that current position to be relative to the curr_marker.pose

                # get position and orientation of the gripper in the base link 
                position, orientation = self._tf_listener.lookupTransform('/base_link', '/wrist_roll_link', rospy.Time(0))
                base_link_T_wrist = Pose(Point(*position), Quaternion(*orientation))

                # The transformation from the base_link to this tags frame is just the tags pose
                base_link_T_tags = copy.deepcopy(marker.pose)

                tags_T_wrist = fetch_api.transform(fetch_api.inverse_pose(base_link_T_tags.pose), base_link_T_wrist)

                new_pose = PoseStamped(pose=tags_T_wrist)
                new_pose.header.frame_id = frame_id
        else:
            # user is trying to define frame_id in some arbitrary frame
            # assume base_link for now
            if frame_id == 'base_link':
                position, orientation = self._tf_listener.lookupTransform('/base_link', '/wrist_roll_link', rospy.Time(0))
                base_link_T_wrist = Pose(Point(*position), Quaternion(*orientation))
                new_pose = PoseStamped(pose=base_link_T_wrist)
                new_pose.header.frame_id = frame_id
            else:
                print 'Please use base_link'
                return
        curr_program.add_step(ProgramStep(new_pose))
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
            if self._curr_markers is None:
                print "No markers exist"
                return
            poses = self._programs[program_name].calc_poses(self._curr_markers)
            print 'poses ', poses
            for i, pose in enumerate(poses):
                error = self._arm.move_to_pose(pose, allowed_planning_time=15.0)
                if error is not None:
                    print "{} failed to run at step #{}".format(program_name, i+1)
                    return
                rospy.sleep(1.5)


    @property
    def programs(self):
        return self._programs
