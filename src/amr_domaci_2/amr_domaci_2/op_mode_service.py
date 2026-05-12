import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from std_srvs.srv import SetBool

class OpModeService(Node):

    def __init__(self):
        super().__init__('choose_op_mode')

        """
        Radni režimi
        False - manuelni režim, kontrola preko tastature
        True - automatski režim, zadavanje cilja"""
        self.op_mode = False

        self.set_op_mode_srv = self.create_service(SetBool, 'set_op_mode', self.callback_set_op_mode)

        self.active_mode_pub = self.create_publisher(Bool, 'set_op_mode/active_mode', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)
        
    def callback_set_op_mode(self, request, response):
   
        if request.data:
            self.op_mode = True
            response.message = "Režim rada promenjen u automatski"
        else:
            self.op_mode = False
            response.message = "Režim rada promenjen u manuelni"

        response.success = True

        self.get_logger().info(response.message)
        return response
    
    def timer_callback(self):
        msg = Bool()
        msg.data = self.op_mode
        self.active_mode_pub.publish(msg)
        
    

init_msg = """
Promena režima rada
Uneti True za perbacivanje u automatski režim, a False za prebacivanje u manuelni režim.
Promenu vršiti upisom sledeće komande:
ros2 service call /set_op_mode std_srvs/srv/SetBool \"{data: <True/False>}\" '
"""

def main():
    rclpy.init()
    print(init_msg)

    op_mode = OpModeService()

    try:
        rclpy.spin(op_mode)
    except KeyboardInterrupt:
        op_mode.destroy_node()


if __name__ == '__main__':
    main()