#! /usr/bin/env python

from moveit_python import PlanningSceneInterface
import fetch_api
import rospy


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def print_usage():
    print 'Usage: rosrun applications a5_obstacles.py'
    print 'Drive the robot until the PlanningScene lines up with the point cloud.'


def main():
    rospy.init_node('a5_obstacles')
    wait_for_time()

    planning_scene = PlanningSceneInterface('base_link')
    planning_scene.clear()
    planning_scene.removeCollisionObject('top_shelf')
    planning_scene.removeCollisionObject('bottom_shelf')
    planning_scene.addBox('top_shelf', 0.32, 1, 0.4, 0.99, 0, 1.64)
    planning_scene.addBox('bottom_shelf', 0.5, 1, 1.09, 0.9, 0, 0.545)

    rospy.sleep(2)


if __name__ == '__main__':
    main()