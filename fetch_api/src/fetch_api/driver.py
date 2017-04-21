import copy
import math
import util
import rospy
class Driver(object):
    def __init__(self, base):
         self.goal = None # position
         self._base = base

    def start(self):
        state = 'turn'
        goal = None
        desired_distance = None
        desired_angular_distance = None
        start_position = None
        while True:

            # Check if someone changed self.goal from outside
            if goal != self.goal:
                print 'changing goal'
                goal = copy.deepcopy(self.goal)
                # TODO: restart the turn/move sequence
                current_pose = self._base.get_current_pose()
                print current_pose
                print goal
                start_position = current_pose.position
                start_orientation = current_pose.orientation
                # Set this to how far the robot should move once pointed in the right direction
                desired_distance = util.distance(start_position.x, start_position.y, goal.x, goal.y)
                
                desired_yaw_rads = math.atan2(goal.y - start_position.y, goal.x - start_position.x)
                start_yaw_rads = util.quaternion_to_yaw(start_orientation)[0]
                desired_angular_distance = desired_yaw_rads - start_yaw_rads

                if desired_angular_distance > math.pi:
                    desired_angular_distance = desired_angular_distance - 2 * math.pi
                elif desired_angular_distance < -1 * math.pi:
                    desired_angular_distance = 2 * math.pi + desired_angular_distance


                direction = -1 if desired_angular_distance < 0 else 1
                state = 'turn'

            if goal == None:
                continue

            current_pose = self._base.get_current_pose()
            current_position = current_pose.position
            current_orientation = current_pose.orientation
            if state == 'turn':
                # TODO: Compute how much we need to turn to face the goal 
                remaining_angle = math.fabs(desired_angular_distance) - math.fabs(util.quaternion_to_yaw(current_orientation)[0] - start_yaw_rads)

                angular_speed = max(0.25, min(1, remaining_angle)) 
                if remaining_angle > 0:
                    direction = -1 if desired_angular_distance < 0 else 1
                    self._base.move(0, direction * remaining_angle)
                else:
                    state = 'move'
                    self._base.stop()

            if state == 'move':
                # TODO: Compute how far we have moved and compare that to desired_distance
                # Make sure that the robot has the ability to drive backwards if it overshoots
                curr_distance = util.distance(start_position.x, start_position.y, current_position.x, current_position.y)
                if  curr_distance < math.fabs(desired_distance):
                    # TODO: possibly adjust speed to slow down when close to the goal
                    threshold_distance = 1.2 #when we should start slowing down
                    faster_speed = 0.1
                    slower_speed = 0.05
                    direction = -1 if desired_distance < 0 else 1
                    changing_dist = math.fabs(desired_distance - curr_distance)
                    if changing_dist >= threshold_distance:
                    	self._base.move(direction * faster_speed, 0)
                    else:
                    	self._base.move(direction * slower_speed, 0)
                    print changing_dist
            rospy.sleep(0.1)