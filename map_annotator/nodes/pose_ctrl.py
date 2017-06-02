import pickle
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from visualization_msgs.msg import Marker

POSE_FILE = '/home/team1/catkin_ws/src/cse481c/map_annotator/nodes/poses'
SUB_NAME = 'amcl_pose'
PUB_NAME = 'move_base_simple/goal'

class PoseController(object):
    def __init__(self, pose_file=POSE_FILE):
        print 'making pose controller'
        self._pose_sub = rospy.Subscriber(SUB_NAME,
                                          PoseWithCovarianceStamped, 
                                          callback=self._pose_callback)
        # self._pose_pub = rospy.Publisher(PUB_NAME,
        #                                  PoseStamped,
        #                                  queue_size=10)

        self._pose_action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self._pose_action_client.wait_for_server()

        self._pose_file = pose_file
        self._poses = self._read_in_poses()
        self._curr_pose = None
        self._marker_publisher = rospy.Publisher('visualization_marker', Marker, latch=True, queue_size=10)
        rospy.sleep(0.5)
        self._publish_location_markers()

    def __str__(self):
        if self._poses:
            return "Poses:\n" "\n".join(["{}:\n{}".format(name, pose) for name, pose in self._poses.items()])
        else:
            return "No poses"

    def _make_location_marker(self, pose):
        box_marker = Marker(type = Marker.CUBE)
        box_marker.pose = pose.pose.pose
        box_marker.ns = "nav"
        box_marker.header.frame_id = pose.header.frame_id
        box_marker.scale.x = 0.45
        box_marker.scale.y = 0.45
        box_marker.scale.z = 0.45
        box_marker.color.r = 0.0
        box_marker.color.g = 0.5
        box_marker.color.b = 0.5
        box_marker.color.a = 1.0
        print box_marker
        return box_marker

    def _make_text_marker(self, name, pose):
        text_marker = Marker(type = Marker.TEXT_VIEW_FACING)
        text_marker.pose = pose.pose.pose
        text_marker.ns = "nav"
        text_marker.header.frame_id = pose.header.frame_id
        text_marker.scale.x = 0.45
        text_marker.scale.y = 0.45
        text_marker.scale.z = 0.45
        text_marker.color.r = 0.0
        text_marker.color.g = 0.5
        text_marker.color.b = 0.5
        text_marker.color.a = 1.0
        text_marker.text = name
        print 'text_marker'
        print text_marker
        return text_marker

    def _publish_location_markers(self):
        for k, v in self._poses.items():
            self._marker_publisher.publish(self._make_location_marker(v))
            self._marker_publisher.publish(self._make_text_marker(k, v))


    def _read_in_poses(self):
        try:
            with open(self._pose_file, 'rb') as file:
                return pickle.load(file)
        except IOError:
            return {}

    def _write_out_poses(self):
        with open(self._pose_file, 'wb') as file:
            pickle.dump(self._poses, file)
        self._publish_location_markers()

    def _pose_callback(self, msg):
        self._curr_pose = msg

    def save_pose(self, pose_name):
        if not self._curr_pose:
            print "No pose available"
            return
        print "Saving pose {} as current position".format(pose_name)
        self._poses[pose_name] = self._curr_pose
        self._write_out_poses()

    def update_pose(self, pose_name, pose):
        if pose_name not in self._poses:
            return
        self._poses[pose_name].pose.pose = pose
        self._write_out_poses()

    def delete_pose(self, pose_name):
        if pose_name in self._poses:
            del self._poses[pose_name]
            print "Pose {} deleted".format(pose_name)
            self._write_out_poses()
        else:
            print "Pose name {} does not exist".format(pose_name)

    def rename_pose(self, pose_name, pose_name_new):
        if pose_name in self._poses:
            self._poses[pose_name_new] = self._poses[pose_name]
            del self._poses[pose_name]
            print "Pose {} renamed to {}".format(pose_name, pose_name_new)
            self._write_out_poses()
        else:
            print "Pose name {} does not exist".format(pose_name)

    def edit_pose(self, pose_name, pose):
        if pose_name in self._poses:
            self._poses[pose_name] = pose
            self._write_out_poses()

    @property
    def poses(self):
        return self._poses

    @property
    def curr_pose(self):
        return self._curr_pose

    def move_to_pose(self, pose_name):
        #TODO change this to use navigation goals
        # move_base_simple/goal
        if pose_name in self._poses:
            msg = PoseStamped()
            msg.header = self._poses[pose_name].header
            msg.pose = self._poses[pose_name].pose.pose
            goal = MoveBaseGoal()
            goal.target_pose = msg
            self._pose_action_client.send_goal(goal)
            self._pose_action_client.wait_for_result()

            return self._pose_action_client.get_state() == GoalStatus.SUCCEEDED

