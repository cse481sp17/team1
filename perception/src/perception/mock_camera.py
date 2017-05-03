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
          #      print msg.header
         #       return PointCloud2(header=msg.header, height=msg.height, width=msg.width, fields=msg.fields, is_bigendian=msg.is_bigendian, point_step=msg.point_step, row_step=msg.row_step, is_dense=msg.is_dense, data=msg.data)
                return msg

            return None
if __name__ == '__main__':
    camera = MockCamera()
    camera.read_cloud('/home/team1/catkin_ws/src/cse481c/perception/src/clowd.bag')
