#!/usr/bin/env python
import rospy, copy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import *
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from std_msgs.msg import ColorRGBA
import fetch_api
from gripper_teleop_util import *
import tf.transformations as tft
from moveit_msgs.msg import OrientationConstraint
from moveit_python import PlanningSceneInterface

class GripperTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server
        self.constraint = None

    def start(self):
        pose = Pose(orientation=Quaternion(0,0,1,1))
        gripper_im = create_gripper_interactive_marker(pose, pregrasp=False)

        self._im_server.insert(gripper_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()

    def handle_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.menu_entry_id == GRIPPER_POSE_ID:
                ps = create_pose_stamped(feedback.pose)
                self._arm.move_to_pose(ps, orientation_constraint = self.constraint)
            elif feedback.menu_entry_id == OPEN_GRIPPER_ID:
                self._gripper.open()
            elif feedback.menu_entry_id == CLOSE_GRIPPER_ID:
                self._gripper.close()
        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            gripper_im = self._im_server.get(feedback.marker_name)
            ps = create_pose_stamped(feedback.pose)
            for marker in gripper_im.controls[0].markers:
                if self._arm.compute_ik(ps):
                    marker.color = GREEN
                else:
                    marker.color = RED
            self._im_server.erase(feedback.marker_name)
            self._im_server.insert(gripper_im)
            self._im_server.applyChanges()

class GripperTeleopDown(GripperTeleop):
    def __init__(self, arm, gripper, im_server):
        super(GripperTeleopDown, self).__init__(arm, gripper, im_server)
        self._table_sub = rospy.Subscriber('/visualization_marker', Marker, self.table_callback)
        self._planning_scene = PlanningSceneInterface('base_link')

    def table_callback(self, msg):
        if msg.ns == 'table':
            self._planning_scene.removeCollisionObject('table')
            # TODO: use the msg.pose.orientation to conver the scale directly 
            # instead of manually swapping x and y
            self._planning_scene.addBox('table', msg.scale.y, msg.scale.x, msg.scale.z, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        if msg.ns == 'tray handle':
            print(msg.pose)
            #handle_im = create_handle_interactive_marker(create_pose_stamped(msg.pose).pose, msg.scale)
            pose = create_pose_stamped(msg.pose).pose
            pose.orientation = self._constraint_pose.orientation
            pose.position.z += 0.3
            handle_im = create_gripper_interactive_marker(pose, 'handle', False, False, False)
            self._im_server.insert(handle_im, feedback_cb=self.handle_feedback)
            self._im_server.applyChanges()

    def start(self):
        mat = tft.identity_matrix()
        mat[:,0] = np.array([0,0,-1,0])
        mat[:,2] = np.array([1,0,0,0])
        o = tft.quaternion_from_matrix(mat)
        print(mat)
        self._constraint_pose = Pose(orientation=Quaternion(*o))
        print(self._constraint_pose)
        gripper_im = create_gripper_interactive_marker(self._constraint_pose, pregrasp=False, rotation_enabled=False)

        self._im_server.insert(gripper_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()

        oc = OrientationConstraint()
        oc.header.frame_id = 'base_link'
        oc.link_name = 'gripper_link'
        oc.orientation = self._constraint_pose.orientation
        oc.weight = 1.0
        oc.absolute_z_axis_tolerance = 0.1
        oc.absolute_x_axis_tolerance = 0.1
        oc.absolute_y_axis_tolerance = 0.1

        self.contraint = oc

class AutoPickTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server

    def start(self):
        pose = Pose(orientation=Quaternion(0,0,0,1))
        gripper_im = create_gripper_interactive_marker(pose)

        self._im_server.insert(gripper_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()

    def handle_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            if feedback.menu_entry_id == GRIPPER_POSE_ID:
                gripper_im = self._im_server.get(feedback.marker_name)
                marker_pre = gripper_im.controls[0].markers[3]
                marker_grasp = gripper_im.controls[0].markers[0]
                marker_lift = gripper_im.controls[0].markers[6]

                # Open gripper
                self._gripper.open()

                # Move to pregrasp
                ps = create_pose_stamped(feedback.pose)
                ps.pose = pose_in_marker_frame(marker_pre, ps.pose)
                self._arm.move_to_pose(ps)
                rospy.sleep(1)

                # Move to grasp
                ps = create_pose_stamped(feedback.pose)
                ps.pose = pose_in_marker_frame(marker_grasp, ps.pose)
                self._arm.move_to_pose(ps)
                rospy.sleep(1)

                # Close gripper
                self._gripper.close()
                rospy.sleep(1)

                # Lift
                ps = create_pose_stamped(feedback.pose)
                ps.pose = pose_in_marker_frame(marker_lift, ps.pose)
                self._arm.move_to_pose(ps)

            elif feedback.menu_entry_id == OPEN_GRIPPER_ID:
                self._gripper.open()
            elif feedback.menu_entry_id == CLOSE_GRIPPER_ID:
                self._gripper.close()
        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            gripper_im = self._im_server.get(feedback.marker_name)
            marker_pre = gripper_im.controls[0].markers[3]
            marker_grasp = gripper_im.controls[0].markers[0]
            marker_lift = gripper_im.controls[0].markers[6]

            # Pregrasp
            ps = create_pose_stamped(feedback.pose)
            ps.pose = pose_in_marker_frame(marker_pre, ps.pose)
            possible = self._arm.compute_ik(ps)
            for marker in gripper_im.controls[0].markers[3:6]:
                if possible:
                    marker.color = GREEN
                else:
                    marker.color = RED

            # Grasp
            ps = create_pose_stamped(feedback.pose)
            ps.pose = pose_in_marker_frame(marker_grasp, ps.pose)
            possible = self._arm.compute_ik(ps)
            for marker in gripper_im.controls[0].markers[:3]:
                if possible:
                    marker.color = GREEN
                else:
                    marker.color = RED

            # Lift
            ps = create_pose_stamped(feedback.pose)
            ps.pose = pose_in_marker_frame(marker_lift, ps.pose)
            possible = self._arm.compute_ik(ps)
            for marker in gripper_im.controls[0].markers[6:9]:
                if possible:
                    marker.color = GREEN
                else:
                    marker.color = RED

            self._im_server.erase(feedback.marker_name)
            self._im_server.insert(gripper_im)
            self._im_server.applyChanges()


def main():
    # Initialize the interactive marker server for the gripper
    rospy.init_node('gripper_demo')
    im_server = InteractiveMarkerServer('gripper_im_server', q_size=2)
    auto_pick_im_server = InteractiveMarkerServer('auto_pick_im_server', q_size=2)
    down_im_server = InteractiveMarkerServer('down_gripper_im_server', q_size=2)
    arm = fetch_api.Arm()
    gripper = fetch_api.Gripper()

    teleop = GripperTeleop(arm, gripper, im_server)
    auto_pick = AutoPickTeleop(arm, gripper, auto_pick_im_server)
    down_teleop = GripperTeleopDown(arm, gripper, down_im_server)
    teleop.start()
    auto_pick.start()
    down_teleop.start()
    rospy.spin()

if __name__ == '__main__':
    main()
