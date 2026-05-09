import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class DriveRobot(Node):
    def __init__(self):
        super().__init__('drive_two_meters_odom')

        # Publisher: nas cvor salje komande brzine robotu
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber: nas cvor cita odometriju robota
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Stanje odometrije
        self.odom_received = False
        self.previous_x = None
        self.previous_y = None
        self.distance_travelled = 0.0

        # Parametri zadatka
        self.speed = 0.2
        self.target_distance = 2.0

        # Stanje izvrsavanja
        self.stopped = False

        # Kontrolna petlja na 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(
            f'Krecem pravo koristeci /odom: v={self.speed} m/s, cilj={self.target_distance} m'
        )

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

    def timer_callback(self):
        msg = Twist()

        if not self.odom_received:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmd_vel_pub.publish(msg)
            self.get_logger().warn('Cekam /odom poruku...')
            return

        if self.distance_travelled < self.target_distance:
            msg.linear.x = self.speed
            msg.angular.z = 0.0
            self.cmd_vel_pub.publish(msg)

            self.get_logger().info(
                f'Predjeni put: {self.distance_travelled:.2f} / {self.target_distance:.2f} m'
            )
        else:
            self.stop_robot()

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


def main(args=None):
    rclpy.init(args=args)

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
