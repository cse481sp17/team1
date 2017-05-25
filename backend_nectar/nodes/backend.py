#!/usr/bin/env python
import rospy
from collections import deque
from perception_msgs.srv import Serving, ServingResponse, ServingRequest
from navigator_msg.srv import Navigation, NavigationRequest, NavigationResponse

class NectarBackend:
    def __init__(self):
        self._order_queue = deque()

        rospy.wait_for_service('serving') # service for placing and grabbing tray
        # calling the serving service via self._serving_server(ServingRequest.RETREIVE) or ServingRequest.PLACE
        self._serving_server = rospy.ServiceProxy('serving', Serving)

        rospy.wait_for_service('navigation') # service for navigation
        # use rossrv show navigator_msg/Navigator to see the possible call values
        self._navigator_server = rospy.ServiceProxy('navigation', Navigation)

        # TODO setup all the other stuff with the frontend communication
        # _order_callback is the callback for the frontend's topic
        # make a subscriber to the frontend
        # self._frontend_sub = rospy.Subcriber(/topic_name, MessageType, self._order_callback)

    # TODO hook this callback into the frontend
    def _order_callback(order):
        self._order_queue.append(order)

    def poll_queue_and_run():
        if len(self._order_queue) == 0:
            return

        cur_order = self._order_queue.popleft()
        run_order(cur_order)

    def run_order(order):
        #TODO run each step of the order

        # go to the chef table
        ret = self._navigator_server(NavigationRequest.CHEF_TABLE)
        if not ret:
            print "ERROR on navigation to chef table"

        ret = self._serving_server(ServingRequest.RETREIVE)
        if not ret:
            print "ERROR in pick up tray"

        # TODO, the navigation location comes from the frontend
        ret = self._navigator_server()
        if not ret:
            print "ERROR in navigation to customer"

        ret = self._serving_server(ServingRequest.PLACE) 
        if not ret:
            print "ERROR in placing tray"


        #TODO communicate to the front end that the order is done

        ret = self._navigator_server(NavigationRequest.CHEF_TABLE)
        if not ret:
            print "ERROR on navigation to chef table"


        return 

    def main_loop():
        while True:
            self.poll_queue_and_run()
            rospy.Sleep()


if __name__ == "__main__":
    nectar = NectarBackend()
    nectar.main_loop()