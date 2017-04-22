import pickle
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

POSE_FILE = 'poses'

def help():
	print 'Welcome to the map annotator!'
	print "Commands:"
	print "\tlist: List saved poses."
	print "\tsave <name>: Save the robot's current pose as <name>. Overwrites if <name> already exists."
	print "\tdelete <name>: Delete the pose given by <name>."
	print "\tgoto <name>: Sends the robot to the pose given by <name>."
	print "\tquit: Exits the program."
	print "\thelp: Show this list of commands"

def read_in_poses():
	try:
		with open(POSE_FILE, 'rb') as file:
			poses = pickle.load(file)
			return poses
	except IOError:
		return {}

def write_out_poses(poses):
	with open(POSE_FILE, 'wb') as file:
		pickle.dump(poses, file)

def get_curr_pose():
	return "Lelillelol"

def move_to_pose(pose, poses):
	if pose in poses:
		pass

def prompt(poses):
	command = raw_input("> ").strip().split()
	if not 1 <= len(command) <= 2:
		print "Invalid command"
		return
	if len(command) == 1:
		command = command[0]
	if len(command) == 2:
		command, name = command

	if command == "list":
		if poses:
			print "Poses: {}".format(poses)
		else:
			print "No poses"

	elif command == "save":
		poses[name] = get_curr_pose()
		write_out_poses(poses)

	elif command == "delete":
		if name in poses:
			del poses[name]
		else:
			print "Pose name {} does not exist".format(name)
		write_out_poses(poses)

	elif command == "goto":
		move_to_pose(name, poses)

	elif command == "quit" or command == "q":
		exit(0)

	elif command == "help":
		help()

	else:
		print "Invalid command"

if __name__ == '__main__':
	help()
	while True:
		poses = read_in_poses()
		try:
			prompt(poses)
		except (KeyboardInterrupt, SystemExit):
			exit(0)
