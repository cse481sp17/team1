#!/usr/bin/env python
import rospy
from collections import deque
from perception_msgs.srv import Serving, ServingResponse, ServingRequest
from navigator_msg.srv import Navigation, NavigationRequest, NavigationResponse
from order_msgs.msg import Order

OrderLocationToNavigationLocation = {Order.COUNTER_AREA_1: NavigationRequest.COUNTER_AREA_1, Order.COUNTER_AREA_2: NavigationRequest.COUNTER_AREA_2}

class NectarBackend:
    def __init__(self):
        self._order_queue = deque()

        rospy.wait_for_service('serving') # service for placing and grabbing tray
        # calling the serving service via self._serving_server(ServingRequest.RETREIVE) or ServingRequest.PLACE
        self._serving_server = rospy.ServiceProxy('serving', Serving)

        rospy.wait_for_service('navigation') # service for navigation
        # use rossrv show navigator_msg/Navigator to see the possible call values
        self._navigator_server = rospy.ServiceProxy('navigation', Navigation)
       
        # _order_callback is the callback for the frontend's topic
        # make a subscriber to the frontend
        self._frontend_sub = rospy.Subscriber('/orders', Order, self._order_callback)

        # make a publisher to the frontend to send errors
        self._frontend_error_pub = rospy.Publisher('/order_failure', Order, queue_size=10)

    def _order_callback(self, msg):
        self._order_queue.append(msg)

    def poll_queue_and_run(self):
        if len(self._order_queue) == 0:
            return

        cur_order = self._order_queue.popleft()
        self.run_order(cur_order)

    def run_order(self, order_msg):
        # run the start pose
        ret = self._serving_server(ServingRequest.START)
        if not ret.success:
            self.error("ERROR in achieving start pose")
            return

        # go to the chef table
        ret = self._navigator_server(NavigationRequest.CHEF_TABLE)
        if not ret.success:
            self.error("ERROR on navigation to chef table", order_msg)
            return

        # retreive the tray
        ret = self._serving_server(ServingRequest.RETREIVE)
        if not ret.success:
            self.error("ERROR in pick up tray", order_msg)
            return

        # move to the customer
        ret = self._navigator_server(OrderLocationToNavigationLocation[order_msg.location])
        if not ret.success:
            self.error("ERROR in navigation to customer", order_msg)
            return

        # place the tray
        ret = self._serving_server(ServingRequest.PLACE)
        if not ret.success:
            self.error("ERROR in placing tray", order_msg)
            return

        # go back to the chef table
        ret = self._navigator_server(NavigationRequest.CHEF_TABLE)
        if not ret.success:
            self.error("ERROR on navigation to chef table", order_msg)
            return

        return 

    def error(self, error_message, order_msg):
        print error_message
        self._frontend_error_pub.publish(order_msg)

    def main_loop(self):
        while True:
            self.poll_queue_and_run()
            rospy.sleep(0.1)


if __name__ == "__main__":
    rospy.init_node('nectar_backend')
    nectar = NectarBackend()
    nectar.main_loop()
