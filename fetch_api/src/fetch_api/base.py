#! /usr/bin/env python

from geometry_msgs.msg import Twist
import rospy
import math
import nav_msgs.msg
import copy
import util

class Base(object):
    """Base controls the mobile base portion of the Fetch robot.

    Sample usage:
        base = fetch_api.Base()
        while CONDITION:
            base.move(0.2, 0)
        base.stop()
    """

    def __init__(self):
        # TODO: Create publisher
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self._odom_sub = rospy.Subscriber('odom', nav_msgs.msg.Odometry, callback=self._odom_callback)
        self.odom_list = []

    def _odom_callback(self, msg):
        self.odom_list.append(msg)

    def get_current_pose(self):
        if len(self.odom_list) > 0:
            return copy.deepcopy(self.odom_list[-1].pose.pose)
        return None

    def go_forward(self, distance, speed=0.1):
        """Moves the robot a certain distance.

        It's recommended that the robot move slowly. If the robot moves too
        quickly, it may overshoot the target. Note also that this method does
        not know if the robot's path is perturbed (e.g., by teleop). It stops
        once the distance traveled is equal to the given distance or more.

        Args:
            distance: The distance, in meters, to move. A positive value
                means forward, negative means backward.
            speed: The speed to travel, in meters/second.
        """
        # TODO: rospy.sleep until the base has received at least one message on /odom
        while (len(self.odom_list) == 0):
            rospy.sleep(0.5)

        current_msg = self.odom_list.pop()
        # TODO: record start position, use Python's copy.deepcopy
        start = copy.deepcopy(current_msg.pose.pose.position)
        rate = rospy.Rate(10)
        # TODO: CONDITION should check if the robot has traveled the desired distance
        current_position = copy.deepcopy(start)

        # TODO: Be sure to handle the case where the distance is negative!
        while util.distance(start.x, start.y, current_position.x, current_position.y) < math.fabs(distance):
            # TODO: you will probably need to do some math in this loop to check the CONDITION
            direction = -1 if distance < 0 else 1
            self.move(direction * speed, 0)
            rate.sleep()
            # TODO: get next current position
            current_position = self.odom_list.pop().pose.pose.position   
        self.stop()

    def turn(self, angular_distance, speed=0.5):
        """Rotates the robot a certain angle.

        Args:
            angular_distance: The angle, in radians, to rotate. A positive
                value rotates counter-clockwise.
            speed: The angular speed to rotate, in radians/second.
        """
        # TODO: rospy.sleep until the base has received at least one message on /odom
        while (len(self.odom_list) == 0):
            rospy.sleep(0.5)
        # TODO: record start position, use Python's copy.deepcopy
        current_msg = self.odom_list.pop()

        start = copy.deepcopy(current_msg.pose.pose.orientation)
        print(start)
        # TODO: What will you do if angular_distance is greater than 2*pi or less than -2*pi?
        direction = -1 if angular_distance < 0 else 1
        angular_distance %= 2 * math.pi * direction
        current_position = copy.deepcopy(start)
        rate = rospy.Rate(10)

        # TODO: CONDITION should check if the robot has rotated the desired amount
        # TODO: Be sure to handle the case where the desired amount is negative!
        start_yaw_rads = util.quaternion_to_yaw(start)[0]
        while math.fabs(util.quaternion_to_yaw(current_position)[0] - start_yaw_rads) < math.fabs(angular_distance):
            # TODO: you will probably need to do some math in this loop to check the CONDITION
            direction = -1 if angular_distance < 0 else 1
            self.move(0, direction * speed)
            rate.sleep()
            current_position = self.odom_list.pop().pose.pose.orientation
        self.stop()


    def move(self, linear_speed, angular_speed):
        """Moves the base instantaneously at given linear and angular speeds.

        "Instantaneously" means that this method must be called continuously in
        a loop for the robot to move.

        Args:
            linear_speed: The forward/backward speed, in meters/second. A
                positive value means the robot should move forward.
            angular_speed: The rotation speed, in radians/second. A positive
                value means the robot should rotate clockwise.
        """
        # TODO: Create Twist msg
        # TODO: Fill out msg
        # TODO: Publish msg
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        self.pub.publish(twist)

    def stop(self):
        """Stops the mobile base from moving.
        """
        # TODO: Publish 0 velocity
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.pub.publish(twist)

