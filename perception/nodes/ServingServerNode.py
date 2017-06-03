#!/usr/bin/env python
from perception_msgs.srv import Serving, ServingRequest
import rospy
from program_ctrl import ProgramController
import fetch_api

class Server:
    TILT_ANGLE = 0.79
    def __init__(self):
        self._program_ctrl = ProgramController()

    # TODO: program retreive and place
    # via the cli.py 
    def serve(self, req):
        print req
        if req.action == ServingRequest.RETREIVE:
            # TODO check for pan_tilt angle
            # and apply it if not close enough
            #self._head.pan_tilt(0, Server.TILT_ANGLE)
            #rospy.sleep(3)
            return self.run_procedure('retrieve')
        elif req.action == ServingRequest.PLACE:
            return self.run_procedure('place')
        elif req.action == ServingRequest.START:
            count = 0
            finish = False
            while count < 5 and not finish:
                finish = self.run_procedure('start')
                count += 1
            return finish
        return False

    def run_procedure(self, name):
        if name not in self._program_ctrl._programs:
            print("{} program does not exist".format(name))
            return False
        return self._program_ctrl.run_program(name)


if __name__ == '__main__':
    rospy.init_node("serving_server")
    server = Server()
    s = rospy.Service('serving', Serving, server.serve)
    rospy.spin()
