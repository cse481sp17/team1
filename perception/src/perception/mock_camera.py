import rosbag
from sensor_msgs.msg import PointCloud2
TOPICS = ['head_camera/depth_registered/points']

class MockCamera(object):
    """A MockCamera reads saved point clouds.
    """
    def __init__(self):
        pass

    def read_cloud(self, path):
        """Returns the sensor_msgs/PointCloud2 in the given bag file.

        Args:
            path: string, the path to a bag file with a single
            sensor_msgs/PointCloud2 in it.

        Returns: A sensor_msgs/PointCloud2 message, or None if there were no
            PointCloud2 messages in the bag file.
        """

        with rosbag.Bag(path) as bag:
            msgs = bag.read_messages()
            for topic, msg, t in msgs:
                return msg
        return None

