#! /usr/bin/env python

import fetch_api
import rospy
from geometry_msgs.msg import *
from std_msgs.msg import Header

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('cart_arm_demo')
    wait_for_time()
    argv = rospy.myargv()

    # Create the arm and safety measures
    arm = fetch_api.Arm()
    def shutdown():
       arm.cancel_all_goals()
       return
    rospy.on_shutdown(shutdown)

    header = Header()
    header.frame_id = 'base_link'

    # Create desired poses

    pose1 = Pose(position=Point(0.042, 0.384, 1.826), orientation=Quaternion(0.173, -0.693, -0.242, 0.657))
    pose2 = Pose(position=Point(0.047, 0.545, 1.822), orientation=Quaternion(-0.274, -0.701, 0.173, 0.635))

    pose1_st = PoseStamped(header, pose1)
    pose2_st = PoseStamped(header, pose2)

    gripper_poses = [pose1_st, pose2_st]

    while True:  
      # Move the arm to each position!
      for pose in gripper_poses:
        error = arm.move_to_pose(pose)

        # Deal with potential errors
        if error is not None:
            rospy.logerr(error)

        rospy.sleep(1)


if __name__ == '__main__':
    main()
