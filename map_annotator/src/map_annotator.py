#!/usr/bin/env python

import pickle
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

POSE_FILE = 'poses'
SUB_NAME = 'amcl_pose'
PUB_NAME = 'move_base_simple/goal'

def help():
    print 'Welcome to the map annotator!'
    print "Commands:"
    print "\tlist: List saved poses."
    print "\tsave <name>: Save the robot's current pose as <name>. Overwrites if <name> already exists."
    print "\tdelete <name>: Delete the pose given by <name>."
    print "\trename <old_name> <new_name>: Rename pose with <old_name> to <new_name>."
    print "\tgoto <name>: Sends the robot to the pose given by <name>."
    print "\tquit: Exits the program."
    print "\thelp: Show this list of commands"

class PoseController(object):
    def __init__(self):
        self._pose_sub = rospy.Subscriber(SUB_NAME,
                                          PoseWithCovarianceStamped, 
                                          callback=self._pose_callback)
        self._pose_pub = rospy.Publisher(PUB_NAME,
                                         PoseStamped,
                                         queue_size=10)
        self._poses = self._read_in_poses()
        self._curr_pose = None

    def __str__(self):
        if self._poses:
            return "Poses:\n" "\n".join(["{}:\n{}".format(name, pose) for name, pose in self._poses.items()])
        else:
            return "No poses"

    def _read_in_poses(self):
        try:
            with open(POSE_FILE, 'rb') as file:
                return pickle.load(file)
        except IOError:
            return {}

    def _write_out_poses(self):
        with open(POSE_FILE, 'wb') as file:
            pickle.dump(self._poses, file)

    def _pose_callback(self, msg):
        self._curr_pose = msg

    def save_pose(self, pose_name):
        if not self._curr_pose:
            print "No pose available"
            return
        print "Saving pose {} as current position".format(pose_name)
        self._poses[pose_name] = self._curr_pose
        self._write_out_poses()

    def delete_pose(self, pose_name):
        if pose_name in self._poses:
            del self._poses[pose_name]
            print "Pose {} deleted".format(pose_name)
            self._write_out_poses()
        else:
            print "Pose name {} does not exist".format(pose_name)

    def rename_pose(self, pose_name, pose_name_new):
        if pose_name in self._poses:
            self._poses[pose_name_new] = self._poses[pose_name]
            del self._poses[pose_name]
            print "Pose {} renamed to {}".format(pose_name, pose_name_new)
            self._write_out_poses()
        else:
            print "Pose name {} does not exist".format(pose_name)

    def move_to_pose(self, pose_name):
        if pose_name in self._poses:
            msg = PoseStamped()
            msg.header = self._poses[pose_name].header
            msg.pose = self._poses[pose_name].pose.pose
            self._pose_pub.publish(msg)

def prompt(pose_ctrl):
    command_args = raw_input("> ").strip().split()
    if not command_args:
        print "Invalid command"
        return

    pose_name, pose_name_new = None, None
    if len(command_args) == 1:
        command = command_args[0]
    if len(command_args) == 2:
        command, pose_name = command_args
    if len(command_args) == 3:
        command, pose_name, pose_name_new = command_args

    if command == "list":
        print str(pose_ctrl)
    elif command == "save" and pose_name:
        pose_ctrl.save_pose(pose_name)
    elif command == "delete" and pose_name:
        pose_ctrl.delete_pose(pose_name)
    elif command == "rename" and pose_name and pose_name_new:
        pose_ctrl.rename_pose(pose_name, pose_name_new)
    elif command == "goto" and pose_name:
        pose_ctrl.move_to_pose(pose_name)
    elif command == "quit" or command == "q":
        exit(0)
    elif command == "help":
        help()
    else:
        print "Invalid command"

def wait_for_time():
    while rospy.Time().now().to_sec() == 0:
        pass

if __name__ == '__main__':
    rospy.init_node('pose_demo', disable_signals=True)
    pose_ctrl = PoseController()
    help()
    while True:
        try:
            prompt(pose_ctrl)
        except (KeyboardInterrupt, SystemExit):
            exit(0)
