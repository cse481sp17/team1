#!/usr/bin/env python
import rospy
from program_ctrl import ProgramController

def help():
    print 'Welcome to the map annotator!'
    print "Commands:"
    print "\tlist: List saved poses"
    print "\tcreate <name>: Create an empty program <name>"
    print "\tsave <name> <frame_id>: Append the robot's current pose relative to <frame_id> to <name>"
    print "\tsavejoint <name> <joint_name>: save the robot's current joint position for joint_name"
    print "\tsavealljoints <name>: save the robot's current joint position"
    print "\tsaveconstraint <name> <frame_id>: same as save but with a down constraint"
    print "\tprepend <name> <frame_id>: same as save, but put the move at the front"
    print "\tdelete <name>: Delete the program given by <name>"
    print "\tdeque <name>: Remove the last step added from <name>"
    print "\tremove <name> <index>: remove the indexed step from name"
    print "\trun <name>: Runs the program given by <name>"
    print "\trelax: Relax the arm"
    print "\tclose: Close the gripper"
    print "\topen: Open the gripper"
    print "\tquit: Exits the program"
    print "\thelp: Show this list of commands"

def prompt(program_ctrl):
    command_args = raw_input("> ").strip().split()
    if not command_args:
        print "Invalid command"
        return

    program_name = None
    command = None
    first_arg = None
    second_arg = None
    third_arg = None

    if len(command_args) == 1:
        command = command_args[0]
    if len(command_args) == 2:
        command, first_arg = command_args
    if len(command_args) == 3:
        command, first_arg, second_arg = command_args
    if len(command_args) == 4:
        command, first_arg, second_arg, third_arg = command_args

    if command == "list":
        print str(program_ctrl)
    elif command == "rename" and first_arg and second_arg:
        program_ctrl.rename(first_arg, second_arg)
    elif command == "close":
        program_ctrl.close()
    elif command == "open":
        program_ctrl.open()
    elif command == "relax":
        program_ctrl.relax_arm()
    elif command == "create" and first_arg:
        program_ctrl.create_program(first_arg)
    elif command == "savejoint" and first_arg and second_arg:
        if third_arg:
            program_ctrl.save_joint(first_arg, second_arg, float(third_arg))
        else:
            program_ctrl.save_joint(first_arg, second_arg)
    elif command == "savealljoints" and first_arg:
        program_ctrl.save_all_joints(first_arg)
    elif command == "save" and first_arg and second_arg:
        program_ctrl.save_program(first_arg, second_arg, append=True)
    elif command == "saveconstraint" and first_arg and second_arg:
        program_ctrl.save_program(first_arg, second_arg, append=True, has_constraint=True)
    elif command == "deque" and first_arg:
        program_ctrl.deque_step(first_arg)
    elif command == "pop" and first_arg:
        program_ctrl.remove_step(first_arg, 0)
    elif command == "prepend" and first_arg and second_arg:
        program_ctrl.save_program(first_arg, second_arg, append=False)
    elif command == "remove" and first_arg and second_arg:
        program_ctrl.remove_step(first_arg, int(second_arg))
    elif command == "delete" and first_arg:
        program_ctrl.delete_program(first_arg)
    elif command == "run" and first_arg:
        program_ctrl.run_program(first_arg)
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
    rospy.init_node('program_demo', disable_signals=True)
    program_ctrl = ProgramController()
    help()
    while True:
        try:
            prompt(program_ctrl)
        except (KeyboardInterrupt, SystemExit):
            exit(0)
