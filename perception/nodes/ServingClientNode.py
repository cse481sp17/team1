#!/usr/bin/env python
import rospy
from perception_msgs.srv import Serving, ServingResponse, ServingRequest

if __name__ == "__main__":
    rospy.wait_for_service('serving')
    try:
        serving = rospy.ServiceProxy('serving', Serving)
        print(serving(ServingRequest.RETREIVE))
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

