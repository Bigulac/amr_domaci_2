import rclpy
from rclpy.node import Node

from std_srvs.srv import SetBool

class Set_Target(Node):
    def __init__(self):
        super().__init__('target_setter')

        self.locked = False

        self.srv = self.create_service(SetBool, 'kuca/brava/zakljucaj', self.handle_lock)

    def handle_lock(self, request, response):
        if request.data:
            self.locked = True
            response.message = "Vrata su zakljucana"
        else:
            self.locked = False
            response.message = "Vrata su otkljucana"

        response.success = True

        self.get_logger().info(response.message)
        return response
    
def main():
    rclpy.init()
    node = Lock()
    rclpy.spin(node)