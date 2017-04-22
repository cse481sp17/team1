import copy
import math
import util
import rospy

def convert_angle_smallest(angle):
    if angle > math.pi:
        angle = angle - 2 * math.pi
    elif angle < -math.pi:
        angle = angle + 2 * math.pi
    return angle

class Driver(object):
    def __init__(self, base):
         self.goal = None # position
         self._base = base

    def start(self):
        goal = copy.deepcopy(self.goal)
        state = 'turn'
        desired_distance = None
        desired_yaw_rads = None
        start_position = None
        while True:
            # Check if someone changed self.goal from outside
            if goal != self.goal:
                goal = copy.deepcopy(self.goal)
                start_pose = self._base.get_current_pose()
                start_position = start_pose.position
                start_orientation = start_pose.orientation
                # Set this to how far the robot should move once pointed in the right direction
                desired_distance = util.distance(start_position.x, start_position.y, goal.x, goal.y) 
                # Set this to the goal angle to reach
                desired_yaw_rads = math.atan2(goal.y - start_position.y, goal.x - start_position.x)
                state = 'turn'

            # we have no goal yet, sleep for a bit and check again
            if goal is None:
                rospy.sleep(0.1)
                continue

            current_pose = self._base.get_current_pose()
            current_position = current_pose.position
            current_orientation = current_pose.orientation
            
            current_yaw = util.quaternion_to_yaw(current_orientation)[0]
            
            # calculate the remaining distance to cover in radians
            # by using the desired_yaw and current_yaw
            remaining_angle = convert_angle_smallest(desired_yaw_rads - current_yaw)

            # if we are not pointing at the goalwithin a tolerance of 0.05 radians
            # adjust
            if math.fabs(remaining_angle) > 0.075:
                state = 'turn'

            if state == 'turn':
                # slow down as we approach our goal
                angular_speed = max(0.25, min(1, math.fabs(remaining_angle))) 
                if math.fabs(remaining_angle) > 0.05:
                    direction = -1 if remaining_angle < 0 else 1
                    self._base.move(0, direction * angular_speed)
                else:
                    state = 'move'
                    self._base.stop()

            if state == 'move':
                # TODO: Compute how far we have moved and compare that to desired_distance
                # Make sure that the robot has the ability to drive backwards if it overshoots
                curr_distance = util.distance(start_position.x, start_position.y, current_position.x, current_position.y)
                remaining_distance = desired_distance - curr_distance              
                if remaining_distance > 0:
                    # TODO: possibly adjust speed to slow down when close to the goal
                    direction = -1 if desired_distance < 0 else 1
                    self._base.move(direction * max(0.05, min(0.5, remaining_distance)), 0)
            rospy.sleep(0.1)


