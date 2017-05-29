#! /usr/bin/env python
from BaseHTTPServer import HTTPServer, BaseHTTPRequestHandler
from SocketServer import ThreadingMixIn
import threading, cgi, json, urlparse
from order_msgs.msg import Order
import rospy

PORT = 8080

FOOD = [
    "Pasta",
    "Hamburger",
    "Burrito",
    "None"
]

SIDES = [
    "Salad",
    "Soup",
    "Fries",
    "None"
]

DESSERTS = [
    "Cookie",
    "Cheesecake",
    "Macarons",
    "None"
]

DRINKS = [
    "Water",
    "Ice tea",
    "Lemonade",
    "None"
]

LOCATIONS = [
    "Counter Area 1",
    "Counter Area 2"
]

ORDERS = []
ORDER_ID = 0

def reply_food_locations_orders(request_handler):
    response = json.dumps({'food': FOOD, 'sides': SIDES, 'dessert': DESSERTS, 'drinks': DRINKS, 'locations': LOCATIONS, 'orders': ORDERS})
    request_handler.send_response(200)
    request_handler.send_header("Content-type", "application/javascript")
    request_handler.send_header("Content-length", len(response))
    request_handler.send_header("Access-Control-Allow-Origin", "*")
    request_handler.end_headers()
    request_handler.wfile.write(response)

def reply_success(request_handler):
    response = "success"
    request_handler.send_response(200)
    request_handler.send_header("Content-type", "text/xml")
    request_handler.send_header("Content-length", len(response))
    request_handler.send_header("Access-Control-Allow-Origin", "*")
    request_handler.end_headers()
    request_handler.wfile.write(response)

def reply_not_found(request_handler):
    response = "not found"
    request_handler.send_response(404)
    request_handler.send_header("Content-type", "text/xml")
    request_handler.send_header("Content-length", len(response))
    request_handler.send_header("Access-Control-Allow-Origin", "*")
    request_handler.end_headers()
    request_handler.wfile.write(response)

def order_failure_callback(msg):
    # append the failed order back to the orders
    print("FAILURE CALLBACK")
    order = {}
    order['id'] = msg.id
    order['location'] = msg.location
    order['sideItem'] = msg.sideItem
    order['drinkItem'] = msg.drinkItem
    order['foodItem'] = msg.foodItem
    order['dessertItem'] = msg.dessertItem
    order['isFail'] = True

    # TODO: add some field that let's the frontend know that it failed and maybe display
    # it different

    # TODO do something like this maybe
    # order['isFail'] = True

    ORDERS.insert(0, order)

class ServerHandler(BaseHTTPRequestHandler):
    def do_OPTIONS(self):
        self.send_response(200, "ok")
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS, DELETE')
        self.send_header("Access-Control-Allow-Headers", "X-Requested-With")

    def do_GET(self):
        print("Receiving GET")
        reply_food_locations_orders(self)

    def do_POST(self):
        print("Receiving POST")
        length = int(self.headers.getheader('content-length'))
        params = cgi.parse_qs(self.rfile.read(length))

        # Convert params to dict, add id, and append to order queue
        order = {k: v[0] for k, v in params.items()}
        global ORDER_ID
        ORDER_ID += 1
        order['id'] = ORDER_ID
        ORDERS.append(order)
        print("Order placed: {}".format(order))
        reply_success(self)

    def do_DELETE(self):
        print("Receiving update")
        id = int(cgi.parse_qs(urlparse.urlparse(self.path).query)['id'][0])
        orders = [o for o in ORDERS if o['id'] == id]
        if not orders:
            print("Order with id {} not found".format(id))
            reply_not_found(self)
            return

        order = orders[0]
        print("Order ready: {}".format(order))

        #TODO: make robot deliver hot steamy pile of food

        # TODO: take order and convert it to an order_msg
        order_msg = Order()
        order_msg.id = order['id']
        order_msg.location = order['location']
        order_msg.sideItem = order['sideItem']
        order_msg.drinkItem = order['drinkItem']
        order_msg.foodItem = order['foodItem']
        order_msg.dessertItem = order['dessertItem']
        NECTAR_ORDER_PUB.publish(order_msg)

        ORDERS.remove(order)
        reply_success(self)


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    pass

rospy.init_node('nectar_web_server_backend')

NECTAR_ORDER_PUB = rospy.Publisher('/orders', Order, queue_size=10)
NECTAR_FAILURE_SUB = rospy.Subscriber('/order_failure', Order, order_failure_callback)

httpd = ThreadedHTTPServer(("", PORT), ServerHandler)
httpd.allow_reuse_address = True

print "Serving at port", PORT
httpd.serve_forever()

