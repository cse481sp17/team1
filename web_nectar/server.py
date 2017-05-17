from BaseHTTPServer import HTTPServer, BaseHTTPRequestHandler
from SocketServer import ThreadingMixIn
import threading, cgi, json

PORT = 8080

FOOD = [
    "apple",
    "orange",
    "bottled water, yum"
]

LOCATIONS = [
    "over here",
    "over there"
]

def send_empty_response(request_handler):
    response = '<?xml version="1.0" encoding="UTF-8"?><Response></Response>'
    request_handler.send_response(200)
    request_handler.send_header("Content-type", "text/xml")
    request_handler.send_header("Content-length", len(response))
    request_handler.send_header("Access-Control-Allow-Origin", "*")
    request_handler.end_headers()
    request_handler.wfile.write(response)

def send_food_and_locations(request_handler):
    response = json.dumps({'food': FOOD, 'locations': LOCATIONS})
    request_handler.send_response(200)
    request_handler.send_header("Content-type", "application/javascript")
    request_handler.send_header("Content-length", len(response))
    request_handler.send_header("Access-Control-Allow-Origin", "*")
    request_handler.end_headers()
    request_handler.wfile.write(response)

def send_successful_delivery(request_handler):
    response = "success"
    request_handler.send_response(200)
    request_handler.send_header("Content-type", "text/xml")
    request_handler.send_header("Content-length", len(response))
    request_handler.send_header("Access-Control-Allow-Origin", "*")
    request_handler.end_headers()
    request_handler.wfile.write(response)

class ServerHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        print "Receiving GET"
        send_food_and_locations(self)

    def do_POST(self):
        print "Receiving POST"
        length = int(self.headers.getheader('content-length'))
        params = cgi.parse_qs(self.rfile.read(length), keep_blank_values=1)
        foodItem = params['foodItem'][0]
        location = params['location'][0]
        print "Food item:", foodItem
        print "Location:", location

        #TODO: make robot deliver hot steamy pile of food

        send_successful_delivery(self)

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    pass

httpd = ThreadedHTTPServer(("", PORT), ServerHandler)
httpd.allow_reuse_address = True

print "Serving at port", PORT
httpd.serve_forever()

