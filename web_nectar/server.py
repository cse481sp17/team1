from BaseHTTPServer import HTTPServer, BaseHTTPRequestHandler
from SocketServer import ThreadingMixIn
import threading, cgi, json

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

class ServerHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        print("Receiving GET")
        reply_food_locations_orders(self)

    def do_POST(self):
        print("Receiving POST")
        length = int(self.headers.getheader('content-length'))
        params = cgi.parse_qs(self.rfile.read(length), keep_blank_values=1)

        # Convert params to dict, add id, and append to order queue
        order = {k: v[0] for k, v in params.items()}
        global ORDER_ID
        ORDER_ID += 1
        order['id'] = ORDER_ID
        ORDERS.append(order)
        print("Order placed: {}".format(order))
        reply_success(self)

    def do_UPDATE(self):
        print("Receiving update")
        #TODO: make robot deliver hot steamy pile of food
        reply_success(self)

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    pass

httpd = ThreadedHTTPServer(("", PORT), ServerHandler)
httpd.allow_reuse_address = True

print "Serving at port", PORT
httpd.serve_forever()

