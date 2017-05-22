from BaseHTTPServer import HTTPServer, BaseHTTPRequestHandler
from SocketServer import ThreadingMixIn
import threading, cgi, json, urlparse

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

        # Remove order from list
        ORDERS.remove(order)
        reply_success(self)

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    pass

httpd = ThreadedHTTPServer(("", PORT), ServerHandler)
httpd.allow_reuse_address = True

print "Serving at port", PORT
httpd.serve_forever()

