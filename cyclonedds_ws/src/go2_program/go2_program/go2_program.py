import json
import time
import rclpy
from rclpy.node import Node
from unitree_api.msg import Request


class Go2ProgramNode(Node):
    def __init__(self):
        super().__init__('go2_program_node')
        self.pub = self.create_publisher(Request, '/api/sport/request', 10)
        self.start_time = time.time()
        self.sit_sent = False          
        self.timer = self.create_timer(0.1, self.publish_move)

    def publish_move(self):
        if self.sit_sent:    #if sit         
            return           #stop the program or you can make it stand up by more codes

        elapsed_seconds = time.time() - self.start_time

        req = Request()
        req.header.identity.id = 0
        req.header.lease.id = 0
        req.header.policy.noreply = True

        if elapsed_seconds < 3.0:
            req.header.identity.api_id = 1008
            vx, vy, vz = 0.3, 0.0, 0.0
        elif elapsed_seconds < 10.0:
            req.header.identity.api_id = 1008
            vx, vy, vz = 0.0, 0.0, 0.5
        elif elapsed_seconds < 13.0:
            req.header.identity.api_id = 1008
            vx, vy, vz = 0.3, 0.0, 0.0
        elif elapsed_seconds < 20.0:
            req.header.identity.api_id = 1008
            vx, vy, vz = 0.0, 0.0, 0.5
        else:
            req.header.identity.api_id = 1009   # Sit
            vx, vy, vz = 0.0, 0.0, 0.0
            self.sit_sent = True               

        if req.header.identity.api_id == 1008: #if moving
            req.parameter = json.dumps({"x": vx, "y": vy, "z": vz})
        self.pub.publish(req)