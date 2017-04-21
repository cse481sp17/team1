#!/usr/bin/env python
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
import fetch_api

if __name__ == "__main__":
    rospy.init_node('something_else')
    base = fetch_api.Base()

    server = InteractiveMarkerServer("simple_server")
    driver = fetch_api.Driver(base)
    marker1 = fetch_api.DestinationMarker(server, 2, 2, 'dest1', driver)
    marker2 = fetch_api.DestinationMarker(server, 1, 0, 'dest2', driver)
    marker3 = fetch_api.DestinationMarker(server, 3, -1, 'dest3', driver)
    server.applyChanges()
    driver.start()