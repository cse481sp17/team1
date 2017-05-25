#!/usr/bin/env python
import rospy
from navigator_msg.srv import Navigation, NavigationRequest, NavigationResponse
import sys

if __name__ == "__main__":
    rospy.wait_for_service('navigation')
    try:
        navigator = rospy.ServiceProxy('navigation', Navigation)
        print(navigator())
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

