#!/usr/bin/env python
import rospy
from collections import deque
from perception_msgs.srv import Serving, ServingResponse, ServingRequest
from navigator_msg.srv import Navigation, NavigationRequest, NavigationResponse
from order_msgs.msg import Order
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
import random

OrderLocationToNavigationLocation = {Order.COUNTER_AREA_1: NavigationRequest.COUNTER_AREA_1, Order.COUNTER_AREA_2: NavigationRequest.COUNTER_AREA_2}

# {0} - entree
# {1} - side item
# {2} - dessert
# {3} - drink
responsesEntree = [
    "Enjoy your steamy {0}",
    "Wow, that {0} has a lot of carbs",
    "{0} is my favorite food. Good job.",
    "Here is your omelette du fro ma ge. I mean {0}"
]

responsesSideItem = [
    "{1} as a side item? Interesting choice.",
    "Our {1} is of the greatest quality.",
    "I am jealous of your {1}. They don't feed me."
]

responsesDessert = [
    "That is a lot of sugar from {2}",
    "Can I have your {2} if you don't eat it?",
    "{2} would be better with some hot fudge syrup",
    "I can tell you are sweet from your choice of {2}. Do you want my I Pee address? You should ping me."
]

responsesDrink = [
    "{3} for humans. Nectar for me.",
    "I am so thirsty. Can I have some of your {3}?",
    "Refills for {3} are free."
]

responsesNone = [
    "Stop wasting my time",
    "Robots have feelings too",
    "Empty plates have to be cleaned",
    "That will be five thousand dollars please",
    "You eat like a robot, want my I Pee address? ping me some time",
]

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

        self._soundhandle = SoundClient()
        rospy.sleep(1)
        self._soundhandle.stopAll()

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
            self.error("ERROR in achieving start pose", order_msg)
            return

        # go to the chef table
        count = 0
        ret = self._navigator_server(NavigationRequest.CHEF_TABLE)
        while not ret.success:
            if count == 5:
                self.error("ERROR on navigation to chef table, order failed!", order_msg)
                return
            print 'ERROR on navigation to chef table, trying again'
            ret = self._navigator_server(NavigationRequest.CHEF_TABLE)
            count += 1
 
        # retreive the tray
        ret = self._serving_server(ServingRequest.RETREIVE)
        if not ret.success:
            self.error("ERROR in pick up tray", order_msg)
            return

        # move to the customer
        ret = self._navigator_server(OrderLocationToNavigationLocation[order_msg.location])
        count = 0
        while not ret.success:
            if count == 5:
                self.error("ERROR in navigation to customer, order failed!", order_msg)
                return
            print 'ERROR on navigation to customer, trying again'
            ret = self._navigator_server(OrderLocationToNavigationLocation[order_msg.location])
            count += 1

        # place the tray
        ret = self._serving_server(ServingRequest.PLACE)
        if not ret.success:
            self.error("ERROR in placing tray", order_msg)
            return
        # TODO: say order
        # order_msg
        choices = []
        if order_msg.foodItem != "None":
            choices.append(responsesEntree)

        if order_msg.sideItem != "None":
            choices.append(responsesSideItem)

        if order_msg.drinkItem != "None":
            choices.append(responsesDrink)

        if order_msg.dessertItem != "None":
            choices.append(responsesDessert)

        if len(choices) == 0:
            choices.append(responsesNone)

        randomList = choices[random.randint(0, len(choices) - 1)]

        randomPhrase = randomList[random.randint(0, len(randomList) - 1)]

        self._soundhandle.say(randomPhrase.format(order_msg.foodItem, order_msg.sideItem, order_msg.dessertItem, order_msg.drinkItem))
        rospy.sleep(3)
        # go back to the chef table
        ret = self._navigator_server(NavigationRequest.CHEF_TABLE)
        count = 0
        while not ret.success:
            print "ERROR on navigation to chef table, trying again"
            if count == 5:
                return
            ret = self._navigator_server(NavigationRequest.CHEF_TABLE)
            count += 1

        ret = self._serving_server(ServingRequest.START)
        if not ret.success:
            print "ERROR in achieving start pose"
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
