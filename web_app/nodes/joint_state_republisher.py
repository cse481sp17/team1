#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from joint_state_reader import JointStateReader

JOINT_NAME = 'torso_lift_joint'

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('joint_state_republisher')
    wait_for_time()
    torso_pub = rospy.Publisher('joint_state_republisher/torso_lift_joint',
                                Float64)
    reader = JointStateReader()
    rospy.sleep(0.5)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # TODO: get torso joint value
        # TODO: publish torso joint value
        torso_pub.publish(reader.get_joint(JOINT_NAME))
        rate.sleep()


if __name__ == '__main__':
    main()
