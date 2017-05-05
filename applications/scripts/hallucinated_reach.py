#! /usr/bin/env python
from geometry_msgs.msg import PoseStamped, Quaternion
import fetch_api
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class ArTagReader(object):
    def __init__(self):
        self.markers = []
    def callback(self, msg):
        self.markers = msg.markers


def main():
    rospy.init_node('hallucinate_demo')
    wait_for_time()

    start = PoseStamped()
    start.header.frame_id = 'base_link'
    start.pose.position.x = 0.5
    start.pose.position.y = 0.5
    start.pose.position.z = 0.75
    arm = fetch_api.Arm()
    arm.move_to_pose(start)
    reader = ArTagReader()
    # Subscribe to AR tag poses, use reader.callback

    sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, reader.callback)
    while len(reader.markers) == 0:
        print 'sleep'
        rospy.sleep(0.1)
    successes = []
    for marker in reader.markers:
        print marker
        ps = PoseStamped()
        ps.pose.position = marker.pose.pose.position
        ps.header.frame_id = 'base_link'
        ps.pose.orientation = Quaternion(0,0,0,1)
        ps.pose.position.x -= 0.3
        error = arm.move_to_pose(ps, allowed_planning_time=30)
        if error is None:
            rospy.loginfo('Moved to marker {}'.format(marker.id))
            successes.append(marker.id)
        else:
            rospy.logwarn('Failed to move to marker {}'.format(marker.id))
    if len(successes) > 0:
        rospy.loginfo('Moved to markers {}!'.format(successes))
    else:
        rospy.logerr('Fail to move to any marker!')


if __name__ == '__main__':
    main()
