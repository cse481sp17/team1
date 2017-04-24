#!/usr/bin/env python

import rospy
from map_annotator_ctrl import PoseController

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
