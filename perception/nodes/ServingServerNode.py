#!/usr/bin/env python
from perception_msgs.srv import Serving, ServingRequest
import rospy
from program_ctrl import ProgramController

PROGRAM_FILE = "programs.p"

class Server:
    def __init__(self):
        self._program_ctrl = ProgramController(PROGRAM_FILE)

    # TODO: program retreive and place
    # via the cli.py 
    def serve(self, req):
        print req
        if req.action == ServingRequest.RETREIVE:
            return self.run_procedure('retreive')
        elif req.action == ServingRequest.PLACE:
            return self.run_procedure('place')
        return False

    def run_procedure(self, name):
        if name not in self._program_ctrl._programs:
            print("{} program does not exist".format(name))
            return False
        #TODO have run_program return a boolean
        # false if the program could not run
        self._program_ctrl.run_program(name)
        return True


if __name__ == '__main__':
    rospy.init_node("serving_server")
    server = Server()
    s = rospy.Service('serving', Serving, server.serve)
    rospy.spin()