#!/usr/bin/env python
import rospy
from program_ctrl import ProgramController

def help():
    print 'Welcome to the map annotator!'
    print "Commands:"
    print "\tlist: List saved poses"
    print "\tcreate <name>: Create an empty program <name>"
    print "\tsave <name> <frame_id>: Append the robot's current pose relative to <frame_id> to <name>"
    print "\tdelete <name>: Delete the program given by <name>"
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
    if len(command_args) == 1:
        command = command_args[0]
    if len(command_args) == 2:
        command, program_name = command_args
    if len(command_args) == 3:
        command, program_name, frame_id = command_args

    if command == "list":
        print str(program_ctrl)
    elif command == "close":
        program_ctrl.close()
    elif command == "open":
        program_ctrl.open()
    elif command == "relax":
        program_ctrl.relax_arm()
    elif command == "create" and program_name:
        program_ctrl.create_program(program_name)
    elif command == "save" and program_name:
        program_ctrl.save_program(program_name, frame_id, True)
    elif command == "prepend" and program_name:
        program_ctrl.save_program(program_name, frame_id, False)
    elif command == "remove" and program_name and frame_id:
        program_ctrl.remove_step(program_name, int(frame_id))
    elif command == "delete" and program_name:
        program_ctrl.delete_program(program_name)
    elif command == "run" and program_name:
        program_ctrl.run_program(program_name)
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
