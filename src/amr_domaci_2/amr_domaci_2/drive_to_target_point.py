import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool



class DriveRobot(Node):
    def __init__(self):
        super().__init__('drive_to_target')

        # Publisher: nas cvor salje komande brzine robotu
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber: nas cvor cita odometriju robota
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.mode_sub = self.create_subscription(
            Bool,
            'set_op_mode/active_mode',
            self.active_mode_callback,
            10
        )

        # Izabran režim rada
        self.active = False

        # Stanje odometrije
        self.odom_received = False
        self.previous_x = None
        self.previous_y = None
        self.distance_travelled = 0.0

        # Parametri zadatka
        self.linear_velocity = None
        self.angular_velocity = None
        self.target_x = None
        self.target_y = None
        self.target_distance = None

        # Kontrolna petlja na 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

        """
        self.get_logger().info(
            f'Krecem pravo koristeci /odom: v={self.speed} m/s, cilj={self.target_distance} m'
        )
        """

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if not self.odom_received:
            self.previous_x = x
            self.previous_y = y
            self.odom_received = True
            self.get_logger().info('Primljena prva /odom poruka. Pocetak merenja puta.')
            return

        dx = x - self.previous_x
        dy = y - self.previous_y

        step_distance = math.sqrt(dx * dx + dy * dy)
        self.distance_travelled += step_distance

        self.previous_x = x
        self.previous_y = y

    def active_mode_callback(self, msg: Bool):
        if msg:
            self.active = True
        else:
            self.active = False

    def timer_callback(self):
        
        if not self.active:
            return
        
        msg = Twist()
        
        """if not self.odom_received:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmd_vel_pub.publish(msg)
            self.get_logger().warn('Cekam /odom poruku...')
            return"""

        if self.distance_travelled < self.target_distance:
            msg.linear.x = self.speed
            msg.angular.z = 0.0
            self.cmd_vel_pub.publish(msg)

            self.get_logger().info(
                f'Predjeni put: {self.distance_travelled:.2f} / {self.target_distance:.2f} m'
            )
        else:
            self.stop_robot()

    def set_target(self, target_x, target_y):
        self.target_x = target_x
        self.target_y = target_y

    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)

        if not self.stopped:
            self.get_logger().info(
                f'Cilj dostignut. Predjeni put: {self.distance_travelled:.2f} m. Robot zaustavljen.'
            )
            self.stopped = True

init_msg = """
Ručni režim - zadavanje cilja
---------------------------
Uneti koordinate ciljne tačke

"""

def main(args=None):
    rclpy.init(args=args)

    print(init_msg)

    target_x = float(input('Koordinata x(format x.x):'))
    target_y = float(input('Koordinata y(format y.y):'))

    node = DriveRobot()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Prekid programa. Zaustavljam robota.')
        stop_msg = Twist()
        node.cmd_vel_pub.publish(stop_msg)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
